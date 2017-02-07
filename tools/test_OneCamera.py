import numpy as np
import cv2
import matplotlib.pylab as plt
import time

port = 0


fig = plt.figure()
cap = cv2.VideoCapture(port)
cap.set(cv2.CAP_PROP_SETTINGS,0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()

    plt.imshow(frame)
    plt.show()
    cv2.waitKey(1)    

    # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()		
