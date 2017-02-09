import numpy as np
import cv2
import time
import matplotlib.pylab as plt

left = 1
right = 2

time_in_ms= 1000/100
#folder = "/home/cwu/Downloads/";
folder = "/home/hillcrest/project/stereo-calibration/calib_imgs/ARC/"

folder = "/home/hillcrest/project/stereo-calibration/calib_imgs/ARC/"
#folder = "D:/vision/stereo-calibration/calib_imgs/ARC/"

fp = open(folder + "timeStamp.txt","w")
        
for counter in range(1,31):
    
    millis = int(round(time.time() * 1000))
    cap1 = cv2.VideoCapture(left)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    cv2.waitKey(100)
    ret, frame1 = cap1.read()
    
    cap1.release()

    cap2 = cv2.VideoCapture(right)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    cv2.waitKey(100)
    ret, frame2 = cap2.read()
    cap2.release()

    #frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    #frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    
    plt.subplot(121)
    plt.imshow(frame1)
    plt.title('left')
    
    plt.subplot(122)
    plt.imshow(frame2)
    plt.title('right')

    plt.show()
    print('another capture', counter)
    cv2.waitKey(100)
    

    cv2.imwrite(folder + "img_left/left_" + str(counter) + ".jpg",  frame1)
    cv2.waitKey(time_in_ms)
    cv2.imwrite(folder + "img_right/right_" + str(counter) + ".jpg", frame2)
    
    fp.write(str(counter)+ ","+ str(millis) + "\n")
    print("the ", counter, " pairs")


cv2.destroyAllWindows()
    
fp.close()
print('All Done \n')
