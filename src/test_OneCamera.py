import numpy as np
import cv2
import matplotlib.pylab as plt
import time


cap = cv2.VideoCapture(0)
time.sleep(2)
toggle = 0

fig = plt.figure()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    if gray.empty():
        print('image not availabe')
    else:
        plt.imshow(gray)
        plt.show()
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()		