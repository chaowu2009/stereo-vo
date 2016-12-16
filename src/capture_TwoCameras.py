import numpy as np
import cv2
import time
import matplotlib.pylab as plt

cap  = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

toggle = 0

fig = plt.figure()
for counter in range(1,5):
    # Capture frame-by-frame

    ret, frame1 = cap.read()

    ret, frame2 = cap2.read()

    gray1 = frame1 #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray2 = frame1 #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #gray1 = cv2.Laplacian(gray1,cv2.CV_64F)
    #gray2 = cv2.Laplacian(gray2,cv2.CV_64F)
    # Display the resulting frame
    plt.subplot(121)
    plt.imshow(gray1)
    plt.title('left')
    
    plt.subplot(122)
    plt.imshow(gray2)
    plt.title('right')

    plt.show()
    
    cv2.imwrite("D:/vision/dataset/calibrationImages/left_" + str(counter) + ".png",  gray1)
    cv2.imwrite("D:/vision/dataset/calibrationImages/right_" + str(counter) + ".png", gray2)

    time.sleep(2)
    print('another capture', counter)
    plt.close()
# When everything done, release the capture
cap.release()
cap2.release()
cv2.destroyAllWindows()