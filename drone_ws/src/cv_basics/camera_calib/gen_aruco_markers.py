# import the necessary packages
import numpy as np
import cv2
import matplotlib.pyplot as plt


arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()

# allocate memory for the output ArUCo tag and then draw the ArUCo
# tag on the output image

tag = np.zeros((300, 300, 1), dtype="uint8")
id = 0
size = 200
marker_image  = cv2.aruco.generateImageMarker(arucoDict, id, size)
fig = plt.figure(0)
plt.imshow(marker_image, cmap='gray')
fig.savefig(f'aruco_4x4_{id}_{size}')
plt.close(fig)
# write the generated ArUCo tag to disk and then display it to our
# screen