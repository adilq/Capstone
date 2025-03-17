#!/usr/bin/python
#

import sys
import jetson.inference
import jetson.utils
import cv2
sys.path.append('/usr/local/lib/python3.6/dist-packages/jetcam-0.0.0-py3.6.egg')
from jetcam.csi_camera import CSICamera


# create the camera and display
camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
display = jetson.utils.glDisplay()

while display.IsOpen():
	img = camera.read()
	display.RenderOnce(img, 1080, 720)