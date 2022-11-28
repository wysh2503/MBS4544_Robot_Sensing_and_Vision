import cv2
import numpy as np

img1 = cv2.imread("AIR_leaflet.jpg", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("holding_leaflet.jpg", cv2.IMREAD_GRAYSCALE)

orb = cv2.ORB_create(nfeatures=1000)
kp1, desc1 = orb.detectAndCompute(img1, None)
kp2, desc2 = orb.detectAndCompute(img2, None)

# Brute-force matcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(desc1,desc2)
matches = sorted(matches, key=lambda x:x.distance)
# for m in matches:
#     print(m.distance)
matching_result = cv2.drawMatches(img1,kp1,img2,kp2,matches[:30],None, flags=2)
# img1 = cv2.drawKeypoints(img1, kp1, None)
# img2 = cv2.drawKeypoints(img2, kp2, None)

cv2.imshow("Image 1", img1)
cv2.imshow("Image 2", img2)
cv2.imshow("matching_result", matching_result)
cv2.waitKey(0)