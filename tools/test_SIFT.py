import numpy as np

import cv2

from matplotlib import pylab as plt

picNumber = 1
filename =  "/home/cwu/project/stereo-calibration/calib_imgs/3/left/left_" + str(picNumber) +".jpg"
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

sift = cv2.xfeatures2d.SIFT_create()
kp, des = sift.detectAndCompute(gray,None)

img = cv2.drawKeypoints(img,kp,None,(255,0,0),4)
cv2.imshow('img',img)
cv2.waitKey()
cv2.imwrite('sift_keypoints.jpg',img)
