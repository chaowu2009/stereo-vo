import numpy as np
import cv2
import matplotlib.pylab as plt
import time

<<<<<<< HEAD
port = 1

=======
port = 0
>>>>>>> aaa15479005b0573ce1688daadc38ac89b544a2c

fig = plt.figure()
cap = cv2.VideoCapture(port)
#cap.set(cv2.CAP_PROP_SETTINGS,0)

WIDTH = 1280
HEIGHT = 720

WIDTH = 640
HEIGHT = 480

cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)

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
