import numpy as np
import cv2
from matplotlib import pylab as plt
import time
import os


# Camera internals
focal_length = 718.856
center = (607.1928, 185.2157)
camera_matrix = np.array(
                         [[focal_length, 0, center[0]],
                         [0, focal_length, center[1]],
                         [0, 0, 1]], dtype = "double"
                         )
dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion 
print "Camera Matrix :\n {0}".format(camera_matrix)

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

T = np.zeros((3,1))
R = np.zeros((3,3))

baseline_distance = 0.52 # meter

for k in range(1,100):
    left_image = leftFolder + str(k).rjust(6, '0') + '.png'
    right_image = rightFolder + str(k).rjust(6, '0') + '.png'

    left_img = cv2.imread(left_image, 0 )
    right_img = cv2.imread(right_image, 0)
	
    orb = cv2.ORB_create()

    kp_left, descriptor_left   = orb.detectAndCompute(left_img, None)
    kp_right, descriptor_right = orb.detectAndCompute(right_img, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck= True)
    matches = bf.match(descriptor_left, descriptor_right)

    matches = sorted(matches, key = lambda x:x.distance)

    img = cv2.drawMatches(left_img, kp_left, right_img, kp_right, matches[:10], 2)

    ax2.imshow(img)
    plt.draw()    
    plt.title("keypoint")

    plt.show()

    #http://docs.opencv.org/3.1.0/d7/d53/tutorial_py_pose.html
	
    plt.pause(0.1)
    #time.sleep(1)


"""    
    img = cv2.imread(left_image)
    
    left_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
    corners = np.int0(corners)

    for m in corners:
    	x,y = m.ravel()
    	cv2.circle(img,(x,y), 8, 255, -1)
    ax1.imshow(img)
    plt.draw()
    plt.title(imgFile + "goodFeaturesToTrack")

    # find keypoints
    #detector = cv2.xfeatures2d.SIFT_create()
    #detector = cv2.xfeatures2d.SURF_create(300)
    #(keypoints, desriptors) = detector.detectAndCompute(gray, None)
    
#    detector = cv2.FastFeatureDetector_create()
#    keypoints= detector.detect(gray, None)
     
    #detector = cv2.FastFeatureDetector_create()
    #brief = cv2.DescriptorExtractor_create("BRIEF")
    #keypoints, descriptors = brief.compute(img, keypoints)
    #keypoints = detector.detect(img, None)

    orb = cv2.ORB_create()
    keypoints = orb.detect(img,None)

    cv2.drawKeypoints(gray,keypoints, img, color = (0,255,0), flags = 1)
"""
