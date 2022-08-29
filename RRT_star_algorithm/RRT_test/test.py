import numpy as np
import cv2
import random


def noCollision(binaryImage, start, end):
    height, _ = binaryImage.shape
    if start[0] == end[0] and start[1] != end[1]:
        coord = [start[1], end[1]]
        if start[1] > end[1]:
            coord = [end[1], start[1]]
        y = np.arange(coord[0], coord[1], 1)
        x = start[0]*np.ones(y.size)
        for i in range(x.size):
            if binaryImage[y[i], x[i]] == 0:
                return False
        return True
    elif start[0] != end[0] and start[1] == end[1]:
        coord = [start[0], end[0]]
        if start[0] > end[0]:
            coord = [end[0], start[0]]
        x = np.arange(coord[0], coord[1], 1)
        y = start[1]*np.ones(x.size)
        for i in range(x.size):
            if binaryImage[y[i], x[i]] == 0:
                return False
        return True
    elif start == end:
        if binaryImage[start[1], start[0]] == 0:
                return False
        return True
    else:
        if start[0] < end[0]:
            x = np.arange(start[0], end[0], 1)
        else: x = np.arange(end[0], start[0], 1)
        y = ((end[1]-start[1])/(end[0]-start[0] ))*(x - start[0])+ start[1]
        for i in range(x.size):
            if y[i] < height:
                if binaryImage[int(y[i]), int(x[i])] == 0:
                    return False
        return True
def SampleFree(binaryImage, height, width):
    while True:
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        if binaryImage[y, x] == 255:
            return [x, y]


imagePath = "D:/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"
image = cv2.imread(imagePath)
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
( _ , binaryImage) = cv2.threshold(grayImage, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
height, width = grayImage.shape
for i in range(10):
    x_start = SampleFree(binaryImage, height, width)
    x_goal = SampleFree(binaryImage, height, width)
    cv2.circle(image, x_start, 5, (0, 0, 255), thickness= 2, lineType= 8)
    cv2.circle(image, x_goal, 5, (0, 0, 255), thickness= 2, lineType= 8)
    cv2.line(image, tuple(x_start), tuple(x_goal), (0, 255, 0), thickness=1)
    print("Start", x_start)
    print("Goal", x_goal)
    print(noCollision(binaryImage,x_start, x_goal))
    cv2.imshow("Image", image)
    cv2.waitKey()