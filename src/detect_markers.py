# reference: https://www.learnopencv.com/augmented-reality-using-aruco-markers-in-opencv-c-python/

import cv2
import numpy as np
 

#Load the dictionary that was used to generate the markers.
dictionary = cv.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
# Initialize the detector parameters using default values
parameters =  cv.aruco.DetectorParameters_create()
 
# Detect the markers in the image
markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)