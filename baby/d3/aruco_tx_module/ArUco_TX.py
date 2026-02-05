import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import struct
import os

# Flask 임포트 시도 및 예외 처리
try:
    from flask import Flask, Response
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False
    print("Warning: Flask not found. Streaming features will be disabled.")

# Flask 앱 설정 (설치된 경우에만 생성)
app = Flask(__name__) if HAS_FLASK else None

# --- 설정 및 데이터 로드 ---
KNOWN_WIDTH = 9.5
IMG_WIDTH = 640
IMG_HEIGHT = 480
SOCKET_PATH = "/tmp/aruco_socket"

# 캘리브레이션 데이터 로드
try:
    with np.load("calib.npz") as data:
        mtx = data['mtx']
        dist = data['dist']
    print("Calibration data loaded.")
except:
    print("Error: calib.npz not found.")
    exit()

# UDS 소켓 서버 설정
if os.path.exists(SOCKET_PATH):
    os.remove(SOCKET_PATH)

server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server_socket.bind(SOCKET_PATH)
server_socket.listen(1)

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

# --- 비디오 스트리밍 및 ArUco 마커 처리 (Flask 전용) ---
if HAS_FLASK:
    def generate_frames():
        print("Waiting for C client connection (Streaming Mode)...")
        client_conn, _ = server_socket.accept()
        print("C client connected.")

        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

        while True:
            success, frame = cap.read()
            if not success: break

            corners, ids, _ = detector.detectMarkers(frame)

            if ids is not None:
                for i in range(len(ids)):
                    # PnP 알고리즘으로 3D 포즈 추정
                    success_pnp, rvec, tvec = cv2.solvePnP(
                        marker_3d_edges, corners[i][0], mtx, dist
                    )

                    if success_pnp:
                        z_distance = tvec[2][0] # Z축 거리
                        c = corners[i][0].astype(int)
                        cx = int(np.mean(c[:, 0]))
                        error = cx - (IMG_WIDTH / 2)

                        # 조향 결정 및 문자 변환
                        steering = "RIGHT" if error > 30 else "LEFT" if error < -30 else "CENTER"
                        steering_char = steering[0].encode() # 'R', 'L', 'C'

                        # IPC 데이터 전송 (13 bytes: int, float, float, char)
                        # < : Little-endian, i : int(4), f : float(4), f : float(4), c : char(1)
                        ipc_data = struct.pack('<iffc', int(ids[i][0]), float(z_distance), float(error), steering_char)
                        try:
                            client_conn.sendall(ipc_data)
                        except (BrokenPipeError, ConnectionResetError):
                            print("Client disconnected.")
                            return

                        # 화면 표시 로직
                        aruco.drawDetectedMarkers(frame, corners)
                        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 5)
                        display_text = f"Z:{z_distance:.1f}cm S:{steering[0]}"
                        cv2.putText(frame, display_text, (c[0][0], c[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.line(frame, (320, 0), (320, 480), (255, 255, 255), 1)
            ret, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        cap.release()
        client_conn.close()

    @app.route('/')
    def index():
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 스트리밍 제외 시 실행되는 메인 로직 ---
def main():
    print("Waiting for C client connection...")
    client_conn, _ = server_socket.accept()
    print("C client connected.")

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    try:
        while True:
            success, frame = cap.read()
            if not success: break

            corners, ids, _ = detector.detectMarkers(frame)

            if ids is not None:
                for i in range(len(ids)):
                    # PnP 알고리즘으로 3D 포즈 추정
                    success_pnp, rvec, tvec = cv2.solvePnP(
                        marker_3d_edges, corners[i][0], mtx, dist
                    )

                    if success_pnp:
                        z_distance = tvec[2][0]
                        c = corners[i][0].astype(int)
                        cx = int(np.mean(c[:, 0]))
                        error = cx - (IMG_WIDTH / 2)

                        steering = "RIGHT" if error > 30 else "LEFT" if error < -30 else "CENTER"
                        steering_char = steering[0].encode() # 'R', 'L', 'C'

                        # IPC 데이터 전송 (13 bytes)
                        ipc_data = struct.pack('<iffc', int(ids[i][0]), float(z_distance), float(error), steering_char)
                        try:
                            client_conn.sendall(ipc_data)
                        except (BrokenPipeError, ConnectionResetError):
                            print("Client disconnected.")
                            return

                        # 터미널 로그 출력 (Flask 대신 실시간 확인용)
                        print(f"ID: {ids[i][0]} | Z: {z_distance:.1f}cm | E: {error:.1f} | S: {steering[0]}", end='\r')

    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        cap.release()
        client_conn.close()
        server_socket.close()

if __name__ == '__main__':
    # Flask가 설치되어 있고, 스트리밍이 필요할 때 app.run() 사용
    # Flask 없이 실행하려면 그냥 main()을 호출하면 됨.
    
    # [방법 1] Flask 스트리밍 사용 시 (HAS_FLASK가 True일 때만 동작)
    # if HAS_FLASK:
    #     app.run(host='0.0.0.0', port=5000, threaded=True)
    # else:
    #     main()

    # [방법 2] 일반적인 터미널 실행 (스트리밍 제외)
    main()