import numpy as np

import cv2

from matplotlib import pylab as plt

# Ref: http://www.pyimagesearch.com/2015/07/16/where-did-sift-and-surf-go-in-opencv-3/

picNumber = 1
filename_1 =  "/home/cwu/project/stereo-calibration/calib_imgs/3/left/left_" + str(picNumber) +".jpg"
filename_2 =  "/home/cwu/project/stereo-calibration/calib_imgs/3/right/right_" + str(picNumber) +".jpg"
img1 = cv2.imread(filename_1)
img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

MIN_MATCH_COUNT = 10

if 0:
    # rotate image 45 degree
    rows,cols = img1.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
    img2 = cv2.warpAffine(img1,M,(cols,rows))
else:
    img2 = cv2.imread(filename_2)
    img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)


sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with ORB
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

matches = flann.knnMatch(des1,des2,k=2)

# store all the good matches as per Lowe's ratio test.
good = []
for m,n in matches:
    if m.distance < 0.3*n.distance:
        good.append(m)

if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()
 
    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

else:
    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    matchesMask = None

draw_params = dict(matchColor = (0,255,0), # draw matches in green color
    singlePointColor = None,
    matchesMask = matchesMask, # draw only inliers
    flags = 2)
     
img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

plt.imshow(img3, 'gray'),plt.show()
