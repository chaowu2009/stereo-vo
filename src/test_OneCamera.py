import numpy as np
import cv2
import matplotlib.pylab as plt
import time

port = 1


fig = plt.figure()

while(True):
    cap = cv2.VideoCapture(port)
    if port == 1:
        port = 0
    else:
        port = 1

    # Capture frame-by-frame
    ret, frame = cap.read()

    plt.imshow(frame)
    plt.show()
    cv2.waitKey(2)    

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()		
