import numpy as np
import cv2
from loadYamlData import *

data = loadYamlData("cam_left_1.yml")
K = data["K"]
D = data["D"]
"""
data = loadYamlData("cam_stereo_1.yml")
K1 = data["K1"]
D1 = data["D1"]
K2 = data["K2"]
D2 = data["D2"]
R = data["R"]
T = data["T"]
E = data["E"]
"""
focal = 837.69
pp = (332.9648, 220.3798)

def convertKeyPoint(kp):
    point =[]
    for k in range(0,len(kp)):
        x,y = kp[k].pt
        point.append([x, y])
    return (np.asarray(point))

def convertMatchedKeyPoint(kp, matches):
    point =[]
    for k in range(0,len(matches)):
        m = matches[k].imgIdx  #get the matched index
        x,y = kp[m].pt         #then get the keypoint index
        point.append([x, y])   #then get the keypoint value
    #print("point", point)
    return (np.asarray(point))

def reconstruct3Dfrom2D():
    # reconstruct 3D from 2D image
 #   P1= K1 * [I3 |0]
 #   P2 = K2 * {R12 |t12]
    print("reconstruct3Dfrom2D")

img1 = cv2.imread("L1.jpg")

img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
#img1 = cv2.undistort(img1, K,D, None)

# Initiate SIFT detector
orb = cv2.ORB_create()

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
missedFrame = 0
DCM = []

img2 = cv2.imread("L6.jpg")
#    img2 = cv2.undistort(img2, K,D, None)
img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)	

# find the keypoints and descriptors with SIFT

kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)
print("len(kp1) = ", len(kp1))
print("len(kp2) = ", len(kp2))
# Match descriptors.
if not kp1 or not kp2 or len(kp1) < 5 or len(kp2)<5:
    missedFrame = missedFrame + 1
    print("could not capture keypoints", missedFrame)
else:
    matches = bf.match(des1,des2)
    print("matched points = ", len(matches))
    #for k  in range(0, len(matches)):
    #    print(matches[k].queryIdx, matches[k].trainIdx, matches[k].imgIdx, matches[k].distance)

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 10 matches.
matchNumber = 499
imgOut = cv2.drawMatches(img1 , kp1, img2, kp2, matches[:matchNumber], 2)

cv2.imshow("matching",imgOut)

#    cv2.drawKeypoints(img2, kp2, imgOut)
#    im_with_keypoints = cv2.drawKeypoints(img2, kp2, np.array([]))
#    cv2.imshow("matching",im_with_kypoints)
#point1 = convertMatchedKeyPoint(kp1, matches)
#point2 = convertMatchedKeyPoint(kp2, matches)
#F,msk = cv2.findFundamentalMat(point1, point2)
#print(DCM)
#retval,R,t, mask = cv2.recoverPose(F, kp1, kp2, focal, pp)
#        print("t= ", t)

cv2.waitKey(0)


cv2.destroyAllWindows()
