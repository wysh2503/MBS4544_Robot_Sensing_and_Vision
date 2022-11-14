import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    blur = cv2. GaussianBlur(frame, (5,5),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    low_orange = np.array([10,70,0])
    high_orange = np.array([70,255,255])
    mask = cv2.inRange(hsv, low_orange,high_orange)

    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # print(contours)
    try:
        for cnt in contours:
            cv2.drawContours(frame, cnt, -1, (0,255,0), 3)
    except: pass

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask",mask)
    if cv2.waitKey(1) & 0xff == 27:
        break

cap.release()
cv2.destroyAllWindows()