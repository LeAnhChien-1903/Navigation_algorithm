import random
import numpy as np
import cv2 
class Node:
    def __init__(self, coordinate):
        self.x = coordinate[0]
        self.y = coordinate[1]
        self.parent = None
def isCollision(binaryImage, nodeStart, nodeEnd):
    height, width = binaryImage.shape
    if nodeStart.x == nodeEnd.x and nodeStart.y != nodeEnd.y:
        coord = [nodeStart.y, nodeEnd.y]
        if nodeStart.y > nodeEnd.y:
            coord = [nodeEnd.y, nodeStart.y]
            y = np.arange(coord[0], coord[1]+1, (coord[1] - coord[0])/1000)
            x = nodeStart.x*np.ones(y.size)
        for i in range(x.size):
            if binaryImage[int(y[i]), int(x[i])] == 0:
                return True
    elif nodeStart.x != nodeEnd.x and nodeStart.y == nodeEnd.y:
        coord = [nodeStart.x, nodeEnd.x]
        if nodeStart.x > nodeEnd.x:
            coord = [nodeEnd.x, nodeStart.x]
            x = np.arange(coord[0], coord[1]+1, (coord[1] - coord[0])/1000)
            y = nodeStart.y*np.ones(x.size)
        for i in range(x.size):
            if binaryImage[int(y[i]), int(x[i])] == 0:
                return True
    elif nodeStart.x == nodeEnd.x and nodeStart.y == nodeEnd.y:
        if binaryImage[int(nodeStart.y), int(nodeStart.x)] == 0:
            return True
    else:
        if nodeStart.x < nodeEnd.x:
            x = np.arange(nodeStart.x, nodeEnd.x+1, (nodeEnd.x - nodeStart.x)/1000)
        else: x = np.arange(nodeEnd.x, nodeStart.x+1, (nodeStart.x - nodeEnd.x)/1000)
        y = ((nodeEnd.y-nodeStart.y)/(nodeEnd.x-nodeStart.x ))*(x - nodeStart.x)+ nodeStart.y
        for i in range(x.size):
            if y[i] < height and x[i] < width:
                if binaryImage[int(y[i]), int(x[i])] == 0:
                    return True
    return False
def generate_random_node(binaryImage, height, width):
    while(True):
        x = np.random.uniform(1, width - 1)
        y = np.random.uniform(1, height - 1)
        if binaryImage[int(round(y, 0)), int(round(x))] == 255:
            return Node((x, y))
imagePath1 = "/home/leanhchien/catkin_ws/src/ros_autonomous_slam/media/my_map.png"
imagePath = "/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"
image = cv2.imread(imagePath1) 
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
( _ ,binaryImage) = cv2.threshold(grayImage, 250, 255, cv2.THRESH_BINARY)
height, width = binaryImage.shape
for i in range(100):
    start = generate_random_node(binaryImage, height, width)
    end = generate_random_node(binaryImage, height, width)
    print(isCollision(binaryImage, start, end))
    imageResult = np.array(image)
    cv2.line(imageResult, (int(start.x), int(start.y)), (int(end.x), int(end.y)), (0, 0, 255), thickness = 1, lineType = 8)
    cv2.imshow("Image", imageResult)
    cv2.waitKey()