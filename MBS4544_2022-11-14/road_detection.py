import cv2
import numpy as np

video = cv2.VideoCapture("road_car_view.mp4")

while True:
    ret, frame = video.read()
    frame_blur = cv2.GaussianBlur(frame,(5,5),0)
    hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
    low_yellow = np.array([25, 95, 120])
    high_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, low_yellow, high_yellow)
    canny = cv2.Canny(mask, 80,150)
    lines = cv2.HoughLinesP(canny, 1, np.pi/180, 50, maxLineGap=50)

    for line in lines:
        x1,y1,x2,y2 = line[0]
        cv2.line(frame, (x1,y1),(x2,y2), (0,255,0),3)

    cv2.imshow("frame", frame)
    cv2.imshow("Canny", canny)
    if cv2.waitKey(10) & 0xff == 27:
        break

cv2.destroyAllWindows()