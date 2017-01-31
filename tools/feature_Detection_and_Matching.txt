import numpy as np
import cv2
from matplotlib import pyplot as plt

leftImgFolder = "D:/vision/dataset/sequences/5/img_left/"
rightImgFolder = "D:/vision/dataset/sequences/5/img_right/"

leftImgFolder  = "D:/vision/dataset/sequences/00/image_0/"
rightImgFolder = "D:/vision/dataset/sequences/00/image_1/"

picFormat = ".png"

imgL = cv2.imread(leftImgFolder + "000001" + picFormat)
imgR = cv2.imread(rightImgFolder + "000001" + picFormat)

imgL = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
imgR = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)

stereo = cv2.StereoSGBM_create(minDisparity =2, numDisparities =16, blockSize=15)
disparity = stereo.compute(imgL,imgR)

plt.figure
plt.subplot(311)
plt.imshow(imgL);
plt.title('left camera')

plt.subplot(312)
plt.imshow(imgR);
plt.title('Right camera')

plt.subplot(313)
plt.imshow(disparity,'gray')
plt.title('disparity image')
plt.show()