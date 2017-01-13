import cv2
vidcap = cv2.VideoCapture('video_rightCamera.avi')
success,image = vidcap.read()
count = 1
success = True
imgFolder = "/home/cwu/project/dataset/images/4/img_right/"
while success:
  success,image = vidcap.read()
  print 'Read a new frame: ', success
  cv2.imwrite(imgFolder +"%d.jpg" % count, image)     # save frame as JPEG file
  count += 1
