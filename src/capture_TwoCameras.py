import numpy as np
import cv2
import time
import matplotlib.pylab as plt

left = 1
right = 2

fig = plt.figure()
for counter in range(1,51):

    cap1  = cv2.VideoCapture(left)
    cap2 = cv2.VideoCapture(right)

    # Capture frame-by-frame
    ret, frame1 = cap1.read()
    ret, frame2 = cap2.read()

    # Display the resulting frame
    plt.subplot(121)
    plt.imshow(frame1)
    plt.title('left')
    
    plt.subplot(122)
    plt.imshow(frame2)
    plt.title('right')

    plt.show()
    
    cv2.imwrite("D:/vision/dataset/calibrationImages/left_" + str(counter) + ".jpg",  frame1)
    cv2.imwrite("D:/vision/dataset/calibrationImages/right_" + str(counter) + ".jpg", frame2)
    time_in_ms= 1000
    cv2.waitKey(time_in_ms)
    print('another capture', counter)
    plt.close()
    # When everything done, release the capture
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()