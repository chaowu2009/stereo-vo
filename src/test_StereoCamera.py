import numpy as np
import cv2

cap  = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

toggle = 0

while(True):
    # Capture frame-by-frame
    if (toggle == 0):
        ret, frame = cap.read()
    else:
        ret, frame = cap2.read()

    # Our operations on the frame come here
    gray = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    if (toggle == 0):
        cv2.imshow('frame',gray)
        toggle = 1
    else:
        cv2.imshow('frame2',gray)
        toggle = 0
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cap2.release()
cv2.destroyAllWindows()