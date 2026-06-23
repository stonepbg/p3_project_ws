import cv2
import numpy as np
from flask import Flask, Response
import mss
import time

app = Flask(__name__)

def generate_frames():
    # mss: 리눅스 환경에서 CPU를 아주 적게 먹는 초고속 화면 캡처 라이브러리
    with mss.mss() as sct:
        monitor = sct.monitors[1]  # 첫 번째 모니터 전체 화면 지정
        
        while True:
            start_time = time.time()
            
            # 1. 화면 캡처 및 색상 변환 (BGRA -> BGR)
            img = np.array(sct.grab(monitor))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            
            # 2. 해상도 강제 축소 (부하 감소를 위해 480x360으로 줄임)
            img_resized = cv2.resize(img_bgr, (480, 360))
            
            # 3. JPEG 인코딩 및 화질 압축 (품질 50%)
            ret, buffer = cv2.imencode('.jpg', img_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            frame = buffer.tobytes()
            
            # 4. 웹 스트리밍 규격(MJPEG)으로 데이터 전송
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            # 5. 초당 10프레임(FPS) 제한을 위한 딜레이 (남는 시간 동안 CPU 휴식)
            elapsed = time.time() - start_time
            time.sleep(max(0, 0.1 - elapsed))

@app.route('/stream')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 외부 PC에서 접속할 수 있도록 0.0.0.0 포트 8080 개방
    app.run(host='0.0.0.0', port=8080, threaded=True)