# reference: https://www.learnopencv.com/augmented-reality-using-aruco-markers-in-opencv-c-python/

import cv2
import numpy as np
 
NUM_MARKERS = 4
N_PIXELS = 200


# Load the predefined dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)

# Generate the marker
for i in range(NUM_MARKERS):
    markerImage = np.zeros((N_PIXELS, N_PIXELS), dtype=np.uint8)
    markerImage = cv2.aruco.drawMarker(dictionary, i, N_PIXELS, markerImage, 1)
    cv2.imwrite("../imgs/marker_{}.png".format(i), markerImage)
