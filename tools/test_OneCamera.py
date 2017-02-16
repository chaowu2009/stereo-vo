import numpy as np
import cv2
import matplotlib.pylab as plt
import time

port = 1


fig = plt.figure()
cap = cv2.VideoCapture(port)
#cap.set(cv2.CAP_PROP_SETTINGS,1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
    cv2.waitKey(1)    

    # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()		
