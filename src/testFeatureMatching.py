import numpy as np
import cv2
from matplotlib import pylab as plt
import time
import os

baseFolder = "/home/cwu/Downloads/dataset/sequences/00/"

left = "image_0/"
right = "image_1/"

leftFolder = baseFolder + left
rightFolder = baseFolder + right

#left camera
img_left = cv2.imread(leftFolder + "000000.png", 0)

#right camera
img_right = cv2.imread(leftFolder + "000010.png", 0)


img = cv2.imread(leftFolder + "000000.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2 , 3, 0.04)

dst = cv2.dilate(dst,None)

img[dst>0.01*dst.max()] = [0,0,255]

#cv2.imshow('dst', img)
#cv2.waitKey()
#cv2.destroyAllWindows()


fig = plt.figure()
ax1 = fig.add_subplot(211)
plt.ion()
plt.show()

ax2 = fig.add_subplot(212)
plt.ion()
plt.show()


for k in range(0,100):
    imgFile = leftFolder + str(k).rjust(6, '0') + '.png'
    img = cv2.imread(imgFile)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
    corners = np.int0(corners)

    for m in corners:
    	x,y = m.ravel()
    	cv2.circle(img,(x,y), 8, 255, -1)
    ax1.imshow(img)
    plt.draw()
    plt.title(imgFile)

    # find keypoints
    sift = cv2.xfeatures2d.SIFT_create()
    (keypoints, desriptors) = sift.detectAndCompute(gray, None)
    cv2.drawKeypoints(gray,keypoints, img)
   
    ax2.imshow(img)
    plt.draw()
    plt.title("keypoint")

    plt.show()


    plt.pause(0.1)
    #time.sleep(1)


"""
stereo = cv2.StereoBM_create(numDisparities = 64, blockSize = 5)

disparity = stereo.compute(img_left, img_right)

plt.subplot(311)
plt.imshow(disparity, 'gray')
plt.title('disparity')

plt.subplot(312)
plt.imshow(img_left, 'gray')
plt.title('left image')

plt.subplot(313)
plt.imshow(img_right, 'gray')
plt.title('right image')


plt.show()
"""

"""
# init SIFT detector
orb = cv2.ORB_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#match descriptors
matches = bf.match(des1, des2)

# sort them in the order of their distance
matches = sorted(matches, key= lambda x:x.distance)

#Draw first 10 matches
flags = 2
img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], flags)

plt.imshow(img3)
plt.show()
"""