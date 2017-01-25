import numpy as np
import cv2
import argparse
import sys

parser = argparse.ArgumentParser(description='Capture Vidoe')
parser.add_argument('videoName', metavar='N', type=str,help='video name')
args = parser.parse_args()
videoName = args.videoName

if (videoName ==''):
   print('please provide video name')
   sys.exit()
   
cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(videoName + '.avi',fourcc, 20.0, (640,480))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
#        frame = cv2.flip(frame,0)

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
