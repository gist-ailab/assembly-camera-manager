import numpy as np
import cv2              
import cv2.aruco as aruco


dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)   # dictionary id
board = aruco.GridBoard_create(5, 7, 0.03, 0.0035, dictionary)
boardImage = aruco.drawPlanarBoard(board, outSize=(2100, 2970), marginSize=150)
cv2.imwrite('board.png', boardImage)


