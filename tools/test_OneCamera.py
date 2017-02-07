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
<<<<<<< HEAD

=======
    cap = cv2.VideoCapture(port)
  
>>>>>>> 5dab2f3a5e0b64d70b0efb0bb68735a02e110869
    # Capture frame-by-frame
    ret, frame = cap.read()

    plt.imshow(frame)
    plt.show()
    cv2.waitKey(1)    

    # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()		
