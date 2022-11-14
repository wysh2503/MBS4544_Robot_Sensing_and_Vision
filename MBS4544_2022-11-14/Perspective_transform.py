import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    cv2.circle(frame, (190,70),5,(0,0,255),-1)
    cv2.circle(frame, (446, 70), 5, (0, 0, 255), -1)
    cv2.circle(frame, (135, 420), 5, (0, 0, 255), -1)
    cv2.circle(frame, (510, 420), 5, (0, 0, 255), -1)

    pts1 = np.float32([[190,70],[446, 70],[135, 420],[510, 420]])
    pts2 = np.float32([[0,0],[148,0],[0,210],[148,210]])
    matrix = cv2. getPerspectiveTransform(pts1,pts2)
    result = cv2.warpPerspective(frame, matrix, (148,210))

    cv2.imshow("Frame", frame)
    cv2.imshow("Transform", result)
    if cv2.waitKey(1) & 0xff == 27:
        break

cap.release()
cv2.destroyAllWindows()