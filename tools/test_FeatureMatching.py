import numpy as np

import cv2

from matplotlib import pylab as plt

# Ref: http://www.pyimagesearch.com/2015/07/16/where-did-sift-and-surf-go-in-opencv-3/

picNumber = 1
filename_1 =  "/home/cwu/project/stereo-calibration/calib_imgs/3/left/left_" + str(picNumber) +".jpg"
filename_2 =  "/home/cwu/project/stereo-calibration/calib_imgs/3/right/right_" + str(picNumber) +".jpg"
img1 = cv2.imread(filename_1)
img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

if 0:
    # rotate image 45 degree
    rows,cols = img1.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),-1,1)
    img2 = cv2.warpAffine(img1,M,(cols,rows))
else:
    img2 = cv2.imread(filename_2)
    img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)


orb = cv2.ORB_create()

# find the keypoints and descriptors with ORB
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors.
matches = bf.match(des1,des2)

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 10 matches.
matchNumber = 10
img3 = cv2.drawMatches(img1 , kp1, img2, kp2, matches[:matchNumber], 2)

plt.imshow(img3),plt.show()
