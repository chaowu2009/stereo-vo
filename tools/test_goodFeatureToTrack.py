import numpy as np

import cv2

from matplotlib import pylab as plt

picNumber = 1
filename =  "/home/cwu/project/stereo-calibration/calib_imgs/3/left/left_" + str(picNumber) +".jpg"
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

maxCornerNumber = 300
minDistance = 0.01
blockSize = 10
corners = cv2.goodFeaturesToTrack(gray,maxCornerNumber,minDistance, blockSize)
corners = np.int0(corners)

dotSize = 4
for i in corners:
    x,y = i.ravel()
    cv2.circle(img,(x,y), dotSize ,255,-1)

plt.imshow(img),plt.show()
