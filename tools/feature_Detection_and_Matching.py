import numpy as np
import cv2
from matplotlib import pyplot as plt

if 1:
    leftImgFolder = "D:/vision/dataset/sequences/5/img_left/"
    rightImgFolder = "D:/vision/dataset/sequences/5/img_right/"
    picFormat = ".jpg"
    imgL = cv2.imread(leftImgFolder + "1" + picFormat)
    imgR = cv2.imread(rightImgFolder + "1" + picFormat)

else:
    leftImgFolder  = "D:/vision/dataset/sequences/00/image_0/"
    rightImgFolder = "D:/vision/dataset/sequences/00/image_1/"
    picFormat = ".png"
    imgL = cv2.imread(leftImgFolder + "000001" + picFormat)
    imgR = cv2.imread(rightImgFolder + "000001" + picFormat)



imgL = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
imgR = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)

# Initiate SIFT detector
orb = cv2.ORB_create()

# find the keypoints and descriptors with SIFT
kpL, des1 = orb.detectAndCompute(imgL,None)
kpR, des2 = orb.detectAndCompute(imgR,None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors.
matches = bf.match(des1,des2)

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 10 matches.
matchNumber = 500
imgOut = cv2.drawMatches(imgL, kpL, imgR, kpR, matches[:matchNumber], 2)

plt.imshow(imgOut),plt.show()