import cv2
import numpy as np

img = cv2.imread("lines.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
canny = cv2.Canny(gray, 80,150)

lines = cv2.HoughLinesP(canny, 1, np.pi/180, 30, maxLineGap=200)
print(lines)
for line in lines:
    x1,y1,x2,y2 = line[0]
    cv2.line(img, (x1,y1),(x2,y2), (0,255,0),3)

cv2.imshow("lines", img)
cv2.imshow("Canny", canny)
cv2.waitKey(0)

cv2.destroyAllWindows()