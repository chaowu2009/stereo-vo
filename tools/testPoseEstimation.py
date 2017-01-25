#!/usr/bin/env python
 
import cv2
import numpy as np
from matplotlib import pylab as plt

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


#http://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/ 
# Read Image
baseFolder = "/home/cwu/Downloads/dataset/sequences/00/"

left = "image_0/"
right = "image_1/"

leftFolder = baseFolder + left
rightFolder = baseFolder + right


fig = plt.figure()
ax1 = fig.add_subplot(211)
plt.ion()
plt.show()

ax2 = fig.add_subplot(212)
plt.ion()
plt.show()

#left camera
img_1 = cv2.imread(leftFolder + "000000.png", 0)
img_2 = cv2.imread(leftFolder + "000001.png", 0)
orb = cv2.ORB_create()


orb = cv2.ORB_create()
feature_1 = orb.detect(img_1, None)
feature_2 = orb.detect(img_2, None)

termcrit=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
cv2.calcOpticalFlowPyrLK(img_1, img_1, points1, points2, status, err, (21,21), 3, termcrit, 0, 0.001);

E = cv2.findEssentialMat(points2, points1, focal, center, RANSAC, 0.999, 1.0, mask);
cv2.recoverPose(E, points2, points1, R, t, focal, center, mask);



for k in range(1,100):
    left_image = leftFolder + str(k).rjust(6, '0') + '.png'
    right_image = rightFolder + str(k).rjust(6, '0') + '.png'

    left_img = cv2.imread(left_image, 0 )
    right_img = cv2.imread(right_image, 0)
    
    #fast feature detection
    orb = cv2.ORB_create()
    kp_left = orb.detect(left_img, None)
    kp_right = orb.detect(right_img, None)
    kp_left = np.asarray(kp_left)
    kp_left_previous = np.asarray(kp_left_previous)
    (success, rotation_vector, translation_vector) = cv2.solvePnP(kp_left_previous, kp_left, camera_matrix, dist_coeffs)
    kp_left_previous = kp_left
    print "Rotation Vector:\n {0}".format(rotation_vector)
    print "Translation Vector:\n {0}".format(translation_vector)
     
    # Project a 3D point (0, 0, 1000.0) onto the image plane.
    # We use this to draw a line sticking out of the nose
     
     
    (nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
     
    for p in image_points:
        cv2.circle(img_left, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
     
     
    p1 = ( int(image_points[0][0]), int(image_points[0][1]))
    p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
     
    cv2.line(img_left, p1, p2, (255,0,0), 2)
     
    # Display image
    ax2.imshow(img_left)
    plt.draw()    
    plt.title("keypoint")
    plt.show()
    plt.pause(5)
