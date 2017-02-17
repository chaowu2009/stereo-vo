import numpy as np
import cv2
from matplotlib import pyplot as plt


picNumber = 1
filename_1 =  "/home/cwu/project/stereo-calibration/calib_imgs/3/left/left_" + str(picNumber) +".jpg"
filename_2 =  "/home/cwu/project/stereo-calibration/calib_imgs/3/right/right_" + str(picNumber) +".jpg"

imgL = cv2.imread(filename_1)
imgR = cv2.imread(filename_2)

imgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
imgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

stereo = cv2.StereoSGBM_create(minDisparity = 5, numDisparities=16, blockSize=15)
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()
