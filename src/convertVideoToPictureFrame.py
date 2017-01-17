import cv2

videoFolder = "/home/cwu/project/dataset/images/5/" 

vidcap = cv2.VideoCapture(videoFolder + 'video_rightCamera.avi')
success,image = vidcap.read()
count = 1
success = True
imgFolder = videoFolder + "img_right/" 
while success:
  success,image = vidcap.read()
  print 'Read a new frame: ', success
  cv2.imwrite(imgFolder +"%d.jpg" % count, image)     # save frame as JPEG file
  count += 1
print("total ", count -1, " frame")  
