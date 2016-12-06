import numpy as np
import cv2
from matplotlib import pyplot as plt


#left camera
img1 = cv2.imread("/home/cwu/Downloads/dataset/sequences/00/image_0/000000.png")

#right camera
img2 = cv2.imread("/home/cwu/Downloads/dataset/sequences/00/image_0/000000.png")

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