import socket
import json
import base64
import numpy as np
import cv2
import time

TCP_IP = "0.0.0.0"
TCP_PORT = 5005

def run_receiver():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((TCP_IP, TCP_PORT))
    server_sock.listen(1)

    print(f"Receiver listening on port {TCP_PORT}...")

    while True:
        conn, addr = server_sock.accept()
        print(f"Connected by {addr}")
        buffer = b""

        try:
            while True:
                # 1. 헤더 읽기
                while len(buffer) < 10:
                    chunk = conn.recv(4096)
                    if not chunk: raise Exception("Connection Closed")
                    buffer += chunk

                header = buffer[:10].decode('utf-8').strip()
                data_len = int(header)
                buffer = buffer[10:]

                # 2. 본문(JSON) 읽기
                while len(buffer) < data_len:
                    chunk = conn.recv(min(data_len - len(buffer), 8192))
                    if not chunk: raise Exception("Connection Closed")
                    buffer += chunk

                frame_data = buffer[:data_len]
                buffer = buffer[data_len:]

                # --- [데이터 처리 및 지연 계산] ---
                recv_time = time.time()
                raw_json = json.loads(frame_data.decode('utf-8'))
                sent_time = float(raw_json.get('timestamp', 0))
                latency_ms = (recv_time - sent_time) * 1000
                detections = raw_json.get('detections', [])

                # --- [터미널 상세 로그 출력] ---
                print("-" * 60)
                print(f"[PTP Time Sync Report]")
                print(f"   - Sent (aig): {sent_time:.6f}")
                print(f"   - Recv (d3g): {recv_time:.6f}")
                print(f"   - Latency   : {latency_ms:7.2f} ms")
                print(f"[Object Detection Result]")
                print(f"   - Count     : {len(detections)} objects")
                
                if detections:
                    for i, det in enumerate(detections):
                        print(f"     └ [{i}] ID: {int(det['id']):>2} | X: {det['x']:>5.1f}, Y: {det['y']:>5.1f} | W: {det['w']:>5.1f}, H: {det['h']:>5.1f}")
                else:
                    print("     (No objects detected)")
                print("-" * 60)

                # 3. 이미지 디코드 및 화면 표시
                img_bytes = base64.b64decode(raw_json['image_base64'])
                img_arr = np.frombuffer(img_bytes, dtype=np.uint8)
                img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    # --- [좌표 보정 로직] ---
                    # 로그 확인 결과: NPU(640x640) -> 이미지(800x480)
                    npu_w, npu_h = 640.0, 640.0
                    img_h, img_w = img.shape[:2]

                    scale_x = img_w / npu_w
                    scale_y = img_h / npu_h

                    for det in detections:
                        # 배율을 적용한 좌표 계산
                        x = int(float(det['x']) * scale_x)
                        y = int(float(det['y']) * scale_y)
                        w = int(float(det['w']) * scale_x)
                        h = int(float(det['h']) * scale_y)
                        
                        # 보정된 위치에 파랑색 박스 그리기 (확인용)
                        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        cv2.putText(img, f"ID:{det['id']}", (x, y - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                    # 지연 시간 표시
                    cv2.putText(img, f"Latency: {latency_ms:.2f} ms", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("PTP Synced Stream", img)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        except Exception as e:
            print(f"\nError: {e}")
        finally:
            conn.close()

if __name__ == "__main__":
    run_receiver()
