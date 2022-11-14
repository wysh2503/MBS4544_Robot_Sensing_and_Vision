import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    blur = cv2.GaussianBlur(frame, (5,5), 0)
    canny = cv2.Canny(blur,50, 150)
    laplacian = cv2.Laplacian(blur, cv2.CV_64F)


    cv2.imshow("Frame", frame)
    cv2.imshow("Blur", blur)
    cv2.imshow("Laplacian", laplacian)
    cv2.imshow("Canny", canny)
    if cv2.waitKey(1) & 0xff == 27:
        break

cap.release()
cv2.destroyAllWindows()