import cv2
print(cv2.__version__)
import numpy as np

img = cv2.imread("AIR_leaflet.jpg")

# sift = cv2.xfeatures2d.SIFT_create()
# surf = cv2.xfeatures2d.SURF_create()
orb = cv2.ORB_create(nfeatures=2000)

# keypoints_sift, descriptors = sift.detectAndCompute(img, None)
# keypoints_surf, descriptors = surf.detectAndCompute(img, None)
keypoints_orb, descriptors = orb.detectAndCompute(img, None)

img = cv2.drawKeypoints(img, keypoints_orb, None)

cv2.imshow("Image", img)
cv2.waitKey(0)