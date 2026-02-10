import argparse
import os
import random
import socket
import struct
import time

# Flask 임포트 시도 및 예외 처리 (스트리밍 모드용)
try:
    from flask import Flask, Response
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False

app = Flask(__name__) if HAS_FLASK else None

# --- 설정 ---
SOCKET_PATH = "/tmp/aruco_socket"
IMG_WIDTH = 640
IMG_HEIGHT = 480
KNOWN_WIDTH = 9.5


def setup_server_socket():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

    server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server_socket.bind(SOCKET_PATH)
    server_socket.listen(1)
    return server_socket


def send_dummy_stream(client_conn, hz, marker_id, dist_cm_min, dist_cm_max,
                      error_px_min, error_px_max, seed):
    if seed is not None:
        random.seed(seed)

    period = 1.0 / hz if hz > 0 else 0.1
    print(
        "Dummy mode: sending random ArUco packets "
        f"(id={marker_id}, hz={hz}, dist_cm=[{dist_cm_min},{dist_cm_max}], "
        f"error_px=[{error_px_min},{error_px_max}])"
    )

    while True:
        dist_cm = random.uniform(dist_cm_min, dist_cm_max)
        error_px = random.uniform(error_px_min, error_px_max)

        if error_px > 30:
            steering_char = b"R"
        elif error_px < -30:
            steering_char = b"L"
        else:
            steering_char = b"C"

        ipc_data = struct.pack('<iffc', int(marker_id), float(dist_cm), float(error_px), steering_char)
        try:
            client_conn.sendall(ipc_data)
        except (BrokenPipeError, ConnectionResetError):
            print("Client disconnected.")
            return

        time.sleep(period)


# --- 카메라 기반 ArUco 처리 ---

def run_camera(stream):
    import cv2
    import cv2.aruco as aruco
    import numpy as np

    # 캘리브레이션 데이터 로드
    try:
        with np.load("calib.npz") as data:
            mtx = data['mtx']
            dist = data['dist']
        print("Calibration data loaded.")
    except Exception:
        print("Error: calib.npz not found.")
        return

    # UDS 소켓 서버 설정
    server_socket = setup_server_socket()

    # ArUco 설정
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # 마커 3D 좌표 (중심 기준)
    marker_3d_edges = np.array([
        [-KNOWN_WIDTH/2,  KNOWN_WIDTH/2, 0],
        [ KNOWN_WIDTH/2,  KNOWN_WIDTH/2, 0],
        [ KNOWN_WIDTH/2, -KNOWN_WIDTH/2, 0],
        [-KNOWN_WIDTH/2, -KNOWN_WIDTH/2, 0]
    ], dtype=np.float32)

    def send_and_annotate(frame, corners, ids, i, rvec, tvec):
        z_distance = tvec[2][0]
        c = corners[i][0].astype(int)
        cx = int(np.mean(c[:, 0]))
        error = cx - (IMG_WIDTH / 2)

        steering = "RIGHT" if error > 30 else "LEFT" if error < -30 else "CENTER"
        steering_char = steering[0].encode()  # 'R', 'L', 'C'

        ipc_data = struct.pack(
            '<iffc', int(ids[i][0]), float(z_distance), float(error), steering_char
        )
        try:
            client_conn.sendall(ipc_data)
        except (BrokenPipeError, ConnectionResetError):
            print("Client disconnected.")
            return None

        aruco.drawDetectedMarkers(frame, corners)
        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 5)
        display_text = f"Z:{z_distance:.1f}cm S:{steering[0]}"
        cv2.putText(frame, display_text, (c[0][0], c[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return z_distance, error, steering

    def run_non_stream():
        print("Waiting for C client connection...")
        client_conn, _ = server_socket.accept()
        print("C client connected.")

        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

        try:
            last_log = None
            while True:
                success, frame = cap.read()
                if not success:
                    break

                corners, ids, _ = detector.detectMarkers(frame)

                if ids is not None:
                    for i in range(len(ids)):
                        success_pnp, rvec, tvec = cv2.solvePnP(
                            marker_3d_edges, corners[i][0], mtx, dist
                        )
                        if not success_pnp:
                            continue

                        result = send_and_annotate(frame, corners, ids, i, rvec, tvec)
                        if result is None:
                            return

                        z_distance, error, steering = result
                        last_log = (ids[i][0], z_distance, error, steering)

                cv2.line(frame, (320, 0), (320, 480), (255, 255, 255), 1)
                if last_log is not None:
                    log_id, log_z, log_error, log_steer = last_log
                    print(
                        f"ID: {log_id} | Z: {log_z:.1f}cm | E: {log_error:.1f} | S: {log_steer[0]}",
                        end='\r'
                    )
        except KeyboardInterrupt:
            print("\nStopping server...")
        finally:
            cap.release()
            client_conn.close()
            server_socket.close()

    def run_stream():
        if not HAS_FLASK:
            print("Warning: Flask not found. Streaming disabled.")
            run_non_stream()
            return

        def generate_frames():
            print("Waiting for C client connection (Streaming Mode)...")
            client_conn, _ = server_socket.accept()
            print("C client connected.")

            cap = cv2.VideoCapture(1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

            try:
                while True:
                    success, frame = cap.read()
                    if not success:
                        break

                    corners, ids, _ = detector.detectMarkers(frame)

                    if ids is not None:
                        for i in range(len(ids)):
                            success_pnp, rvec, tvec = cv2.solvePnP(
                                marker_3d_edges, corners[i][0], mtx, dist
                            )
                            if not success_pnp:
                                continue

                            result = send_and_annotate(frame, corners, ids, i, rvec, tvec)
                            if result is None:
                                return

                    cv2.line(frame, (320, 0), (320, 480), (255, 255, 255), 1)
                    ret, buffer = cv2.imencode('.jpg', frame)
                    if not ret:
                        continue
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            finally:
                cap.release()
                client_conn.close()

        @app.route('/')
        def index():
            return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

        app.run(host='0.0.0.0', port=5000, threaded=True)

    if stream:
        run_stream()
    else:
        run_non_stream()


def main():
    parser = argparse.ArgumentParser(description="ArUco TX module")
    parser.add_argument("--dummy", action="store_true", help="send random dummy data (no camera)")
    parser.add_argument("--hz", type=float, default=10.0, help="dummy send rate (Hz)")
    parser.add_argument("--id", type=int, default=1, help="dummy marker id")
    parser.add_argument("--dist-cm-min", type=float, default=50.0)
    parser.add_argument("--dist-cm-max", type=float, default=200.0)
    parser.add_argument("--error-px-min", type=float, default=-320.0)
    parser.add_argument("--error-px-max", type=float, default=320.0)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--stream", action="store_true", help="use Flask streaming (camera mode only)")
    args = parser.parse_args()

    if args.dummy:
        server_socket = setup_server_socket()
        print("Waiting for C client connection...")
        client_conn, _ = server_socket.accept()
        print("C client connected.")
        try:
            send_dummy_stream(
                client_conn,
                args.hz,
                args.id,
                args.dist_cm_min,
                args.dist_cm_max,
                args.error_px_min,
                args.error_px_max,
                args.seed,
            )
        except KeyboardInterrupt:
            print("\nStopping server...")
        finally:
            client_conn.close()
            server_socket.close()
    else:
        run_camera(args.stream)


if __name__ == '__main__':
    main()
