import numpy as np
import cv2

def reconstruct3Dfrom2D():
    # reconstruct 3D from 2D image
 #   P1= K1 * [I3 |0]
 #   P2 = K2 * {R12 |t12]

   
cap = cv2.VideoCapture(0)
ret, img1 = cap.read()

img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

# Initiate SIFT detector
orb = cv2.ORB_create()

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


while(cap.isOpened()):
    ret, img2 = cap.read()

    img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)	

	
    # find the keypoints and descriptors with SIFT

    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    # Match descriptors.
    matches = bf.match(des1,des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    # Draw first 10 matches.
    matchNumber = 200
    imgOut = cv2.drawMatches(img1, kp1, img2, kp2, matches[:matchNumber], 2)

    cv2.imshow("matching",imgOut)
    
    cv2.waitKey(1)

    #udpate previous frame
    img1 = img2;

cap.release()

cv2.destroyAllWindows()
