import numpy as np
import cv2
import argparse
import sys

LEFT = 0
RIGHT = 1
   
WIDTH = 1280;
HEIGHT = 720;
   
cap_left  = cv2.VideoCapture(LEFT)
cap_right = cv2.VideoCapture(RIGHT)

cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH);
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT);

cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH);
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT);

#cap_left.set(cv2.CAP_PROP_EXPOSURE, inf);
#cap_right.set(cv2.CAP_PROP_EXPOSURE, inf);

frameRate = 60

#cap_left.set(cv2.CAP_PROP_FPS , 60)
#cap_right.set(cv2.CAP_PROP_FPS , 60)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_left  = cv2.VideoWriter("video_leftCamera" + '.avi',fourcc, frameRate, (WIDTH,HEIGHT))
out_right = cv2.VideoWriter("video_rightCamera" + '.avi',fourcc, frameRate, (WIDTH,HEIGHT))

while(cap_left.isOpened() and cap_right.isOpened()):
    ret_left, frame_left  = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if ret_left ==True and ret_right == True:
        # write the flipped frame
        out_left.write(frame_left)
        out_right.write(frame_right)

        cv2.imshow('left',frame_left)
        cv2.imshow('right',frame_right)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap_left.release()
cap_right.release()
out_left.release()
out_right.release()
cv2.destroyAllWindows()
