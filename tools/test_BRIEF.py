import numpy as np

import cv2

from matplotlib import pylab as plt

# Ref: http://www.pyimagesearch.com/2015/07/16/where-did-sift-and-surf-go-in-opencv-3/

picNumber = 1
filename =  "/home/cwu/project/stereo-calibration/calib_imgs/3/left/left_" + str(picNumber) +".jpg"
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

star = cv2.xfeatures2d.StarDetector_create()
# Initiate BRIEF extractor
brief = cv2.xfeatures2d.BriefDescriptorExtractor_create()

# find the keypoints with STAR
kp = star.detect(img,None)

# compute the descriptors with BRIEF
kp, des = brief.compute(img, kp)
img = cv2.drawKeypoints(img,kp,None,(0,255,0),4)


cv2.imshow('img',img)
cv2.waitKey()
print brief.getInt('bytes')
print des.shape
