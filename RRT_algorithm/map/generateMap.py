from dis import dis
from math import dist
import cv2
import numpy as np

imagePath = "/home/leanhchien/Navigation_algorithm/RRT_algorithm/map/map.png"

image = cv2.imread(imagePath)
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
( _ , binaryImage) = cv2.threshold(grayImage, 250, 255, cv2.THRESH_BINARY)
height, width = grayImage.shape
result = []
for y in range(height):
    tempList = []
    for x in range(width):
        if binaryImage[y, x] == 255:
            tempList.append((x, y))
    if len(tempList) > 0:
        result.append(tempList)

y_start = result[0][0][1]
y_end = result[-1][0][1]

x_list_min = []
x_list_max = []
for i in range(len(result)):
    x_list_min.append(result[i][0][0])
    x_list_max.append(result[i][-1][0])

x_start = min(x_list_min)
x_end = max(x_list_max)

imageResult = np.zeros((y_end - y_start + 10, x_end - x_start + 10))
imageTemp = binaryImage[y_start: y_end, x_start:x_end]

resultHeight, resultWidth = imageTemp.shape

for y in range(resultHeight):
    for x in range(resultWidth):
        imageResult[y + 5, x + 5] = imageTemp[y, x]

imageResult = cv2.resize(imageResult, (0, 0), fx = 3, fy = 3)
print(imageResult.shape)
cv2.imshow("Image", imageResult)
cv2.waitKey()