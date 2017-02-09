import numpy as np
import cv2
import time
import matplotlib.pylab as plt

left = 1
right = 2

time_in_ms= 1000/100


fig = plt.figure()
        
for counter in range(1,5):

    cap1 = cv2.VideoCapture(left)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
#    cv2.waitKey(time_in_ms)
    # Capture frame-by-frame
    ret, frame1 = cap1.read()
    cap1.release()
    
    cap2 = cv2.VideoCapture(right)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

    #cv2.waitKey(time_in_ms)
    ret, frame2 = cap2.read()
    cap2.release()
     
    # Display the resulting frame
    plt.subplot(121)
    plt.imshow(frame1)
    plt.title('left')
    
    plt.subplot(122)
    plt.imshow(frame2)
    plt.title('right')

    plt.show()
    cv2.waitKey(time_in_ms)
    print('another capture', counter)
    plt.close()
    # When everything done, release the capture
    

cv2.destroyAllWindows()
    

