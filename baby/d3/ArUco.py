import cv2
import cv2.aruco as aruco
import numpy as np
from flask import Flask, Response

app = Flask(__name__)

# --- 설정값 ---
KNOWN_WIDTH = 9.5      # 마커 실제 크기
FOCAL_LENGTH = 706.0   # 초점 거리
DISTANCE_OFFSET = 0.8  # 오차 보정
IMG_WIDTH = 640
IMG_HEIGHT = 480

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

def generate_frames():
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)

    while True:
        success, frame = cap.read()
        if not success: break
        
        corners, ids, rejected = detector.detectMarkers(frame)
        
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                c = corners[i][0]
                tl = tuple(c[0].astype(int))
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                
                pixel_width = np.linalg.norm(c[0] - c[1])
                distance = ((KNOWN_WIDTH * FOCAL_LENGTH) / pixel_width) + DISTANCE_OFFSET
                error = cx - (IMG_WIDTH / 2)
                steering = "RIGHT" if error > 30 else "LEFT" if error < -30 else "CENTER"

                # 데이터 딕셔너리
                control_data = {
                    "id": int(ids[i][0]),
                    "dist": round(distance, 2),
                    "error": round(error, 1),
                    "steering": steering
                }
                print(f"CONTROL_LOG: {control_data}")

                # 화면 출력용 텍스트
                display_text = f"D:{distance:.1f}cm E:{error:.1f} S:{steering}"
                
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, display_text, (tl[0], tl[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
        cv2.line(frame, (320, 0), (320, 480), (255, 255, 255), 1)
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    cap.release()

@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
