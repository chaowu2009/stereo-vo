import cv2

videoFolder ="D:/vision/dataset/gopro/"
imgFolder = videoFolder + "60HZ/"

vidcap = cv2.VideoCapture(videoFolder + 'gopro_cal_60HZ.MP4')
vidcap.set(cv2.CAP_PROP_FPS,60)    
FPS = vidcap.get(5)
frameNumber = vidcap.get(7)
 
success,image = vidcap.read()
print(success)
count = 1
success = True

while count < frameNumber :
  success,image = vidcap.read()
  print ('Read a new frame: ', success, " frame number: ", count)
  cv2.imwrite(imgFolder +"%d.jpg" % count, image)     # save frame as JPEG file
  count += 1
print("total ", count -1, " frame")  
