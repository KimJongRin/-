import cv2
from datetime import datetime
import time 

def test_cap():
    cap = cv2.VideoCapture(0)    
    while cap.isOpened():
        now = datetime.now().strftime("%y%m%d_%H-%M-%S")
        ret, frame = cap.read()    
        frame = cv2.resize(frame, (1920,1080))                # 카메라 프레임 읽기
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        cv2.imwrite(str(now)+'-0.png', frame) # 프레임을 'photo.png'에 저장
        print("save "+str(now))
        cap.release()
        break



test_cap()
