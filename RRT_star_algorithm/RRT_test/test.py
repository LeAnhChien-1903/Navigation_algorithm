import cv2  
import numpy as np

imagePath = "D:/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1_8_8_40_8.png"
image = cv2.imread(imagePath)
grayImage = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
(thresh, binaryImage) = cv2.threshold(grayImage, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
cv2.imshow("Origin", image)
cv2.imshow("Gray", grayImage)
cv2.imshow("Binary", binaryImage)
cv2.waitKey()
