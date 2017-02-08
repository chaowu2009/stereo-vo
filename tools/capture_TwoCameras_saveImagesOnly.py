import numpy as np
import cv2
import time
import matplotlib.pylab as plt

left = 0
right = 1

time_in_ms= 1000/100
#folder = "/home/cwu/Downloads/";
folder = "/home/cwu/project/data/images/FHD/"


fp = open(folder + "timeStamp.txt","w")
        
for counter in range(1,51):
    
    millis = int(round(time.time() * 1000))
    cap1 = cv2.VideoCapture(left)
    ret, frame1 = cap1.read()
    
    cap2 = cv2.VideoCapture(right)
    ret, frame2 = cap2.read()

    cv2.imwrite(folder + "img_left/left_" + str(counter) + ".jpg",  frame1)
    cv2.imwrite(folder + "img_right/right_" + str(counter) + ".jpg", frame2)
    
    cap1.release()
    cap2.release()    
    fp.write(str(counter)+ ","+ str(millis) + "\n")


cv2.destroyAllWindows()
    
fp.close()
print('All Done \n')
