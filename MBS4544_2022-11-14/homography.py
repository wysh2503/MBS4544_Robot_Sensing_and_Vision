import cv2
import numpy as np

img = cv2.imread("AIR_leaflet.jpg", cv2.IMREAD_GRAYSCALE)
sift = cv2.xfeatures2d.SIFT_create()
kp_img, desc_img = sift.detectAndCompute(img, None)

index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp_grayframe, desc_grayframe = sift.detectAndCompute(grayframe, None)
    matches = flann.knnMatch(desc_img, desc_grayframe, k=2)
    good_points = []
    for m,n in matches:
        if m.distance < n.distance * 0.5:
            good_points.append(m)
    imgX = cv2.drawMatches(img,kp_img,grayframe,kp_grayframe,good_points, grayframe)

    try:
        if len(good_points) > 10:
            query_pts = np.float32([kp_img[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
            train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()
            # Perspective transform
            h, w = img.shape
            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, matrix)
            homography = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
            cv2.imshow("Homography", homography)
        else:
            cv2.imshow("Homography", grayframe)
    except:
        pass

    cv2.imshow("Gray Frame", grayframe)
    cv2.imshow("Leaflet", img)
    cv2.imshow("imgX", imgX)
    if cv2.waitKey(1) & 0xff == 27:
        break

cap.release()
cv2.destroyAllWindows()
