import numpy as np
from cv2 import *
import matplotlib.pylab as plt
import time

port = 1
cap = VideoCapture(port)
"""
CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds or video capture timestamp.
CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
CV_CAP_PROP_FPS Frame rate.
CV_CAP_PROP_FOURCC 4-character code of codec.
CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
CV_CAP_PROP_HUE Hue of the image (only for cameras).
CV_CAP_PROP_GAIN Gain of the image (only for cameras).
CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
CV_CAP_PROP_WHITE_BALANCE Currently not supported
CV_CAP_PROP_RECTIFICATION
"""
print("CV_CAP_PROP_POS_MSEC",     cap.get(CAP_PROP_POS_MSEC))
#print("CV_CAP_PROP_POS_FRAMES", cap.get(CAP_PROP_POS_FRAMES))
print("CV_CAP_PROP_POS_AVI_RATIO", cap.get(CAP_PROP_POS_AVI_RATIO))
print("CV_CAP_PROP_FRAME_WIDTH",  cap.get(CAP_PROP_FRAME_WIDTH))
print("CV_CAP_PROP_FRAME_HEIGHT", cap.get(CAP_PROP_FRAME_HEIGHT))
print("CV_CAP_PROP_FPS",          cap.get(CAP_PROP_FPS))
#print("CV_CAP_PROP_FOURCC",      cap.get(CAP_PROP_FOURCC))
print("CV_CAP_PROP_FORMAT",       cap.get(CAP_PROP_FORMAT))
print("CV_CAP_PROP_MODE",         cap.get(CAP_PROP_MODE))
print("CV_CAP_PROP_BRIGHTNESS",   cap.get(CAP_PROP_BRIGHTNESS))
print("CV_CAP_PROP_CONTRAST",     cap.get(CAP_PROP_CONTRAST))
print("CV_CAP_PROP_SATURATION",   cap.get(CAP_PROP_SATURATION))
print("CV_CAP_PROP_HUE",          cap.get(CAP_PROP_HUE))
print("CV_CAP_PROP_GAIN",         cap.get(CAP_PROP_GAIN))
print("CV_CAP_PROP_EXPOSURE",     cap.get(CAP_PROP_EXPOSURE))
print("CV_CAP_PROP_CONVERT_RGB",  cap.get(CAP_PROP_CONVERT_RGB))
#print("CV_CAP_PROP_WHITE_BALANCE", cap.get(CAP_PROP_WHITE_BALANCE))
print("CV_CAP_PROP_RECTIFICATION", cap.get(CAP_PROP_RECTIFICATION))

cap.release()

