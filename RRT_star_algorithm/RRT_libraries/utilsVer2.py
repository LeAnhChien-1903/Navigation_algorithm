"""
    Utils for collision check
"""
from msilib.schema import Directory
import cv2
import numpy as np
import math
import sys

sys.path.insert(0, 'RRT_libraries')
import env
from node import Node

class Utils:
    def __init__(self, grayImage):
        self.grayImage = grayImage
        self.delta = 0.5
    def collision(self, start, end):
        x = list(np.arange(start.x, end.x, (end.x-start.x/100)))
        y = list(((end.y-start.y)/(end.x-start.x))*(x - start.x)+ start.y)
        for i in range(len(x)):
            if self.grayImage[int(y[i]), int(x[i])] == 0:
                return True
        return False
    def getDistance(self, start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
    def getAngle(self, start, end):
        return math.atan2(end.y - start.y, end.x - start.x)