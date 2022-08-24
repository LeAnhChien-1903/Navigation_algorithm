from copyreg import dispatch_table
from dis import dis
import cv2
import numpy as np
import os
import random
import math
import argparse

class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x, y):
        self.x = x 
        self.y = y
        self.parent_x = []
        self.parent_y = []
class RRT_star:
    def __init__(self, image, eta, start = (20, 20), goal = (450, 250)):
        self.image = image
        self.height, self.width, _ = image.shape
        self.grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        self.start = start
        self.goal = goal
        self.eta = eta
    # return dist and angle between two point
    def dist_and_angle(self, x, y ):
        dist = math.sqrt((y[0] - x[0])**2 + (y[1] - x[1])**2)
        angle = math.atan2(y[1] - x[1], y[0] - x[0])
        return (dist, angle)
    # generate a random point in the free space
    def Sample(self):
        new_x = random.randint(0, self.width)
        new_y = random.randint(0, self.height)
        while (self.grayImage[new_y, new_x] == 0):
            new_x = random.randint(0, self.width)
            new_y = random.randint(0, self.height)
        return (new_x, new_y)
    # return a point z is "closer" to than y than x
    def Steer(self, x, y):
        # get a list of point around points x
        aroundPoints = []
        distanceArray = [] 
        for i in range(-int(self.eta),int(self.eta)):
            for j in range(-int(self.eta),int(self.eta)):
                if 0 <= x[0] + i < self.width and 0 <= x[1] + j <=self.height: 
                    if self.grayImage[x[1]+j, x[0]+i] == 255:
                        tempPoint = (x[0]+i, x[1]+j)
                        distanceX, _ = self.dist_and_angle(x, tempPoint)
                        if distanceX <= self.eta:
                            aroundPoints.append(tempPoint)
                            distanceY, _ = self.dist_and_angle(y, tempPoint)
                            distanceArray.append(distanceY)
        if len(distanceArray)==0:
            return x
        else:
            minDistance = min(distanceArray)
            index = distanceArray.index(minDistance)
            return aroundPoints[index]
    
    # Return nearest neighbor of a point
    def Nearest(self, G, x):
        distanceArray = []
        for i in range(len(G[0])):
            distance = self.dist_and_angle(G[0][i], x)
            distanceArray.append(distance)
        minDistance = min(distanceArray)
        index = distanceArray.index(minDistance)

        return G[0][index]
    # Return a set V' of vertices such V' is subset of V
    def Near(self, G, x, n):
        pass
imagePath = "world2.png"
image = cv2.imread(imagePath)  
    
RRT_star_obj = RRT_star(image, 5) 
print(RRT_star_obj.Steer((50, 50), (20, 20)))
