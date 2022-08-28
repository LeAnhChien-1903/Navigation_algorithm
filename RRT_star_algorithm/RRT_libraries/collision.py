"""
    Utils for collision check
"""
import numpy as np
import sys

sys.path.insert(0, 'RRT_libraries')

class Collision:
    def __init__(self, binaryImage):
        self.binaryImage = binaryImage
    def CollisionFree(self, start, end):
        x = np.arange(start.x, end.x, int((end.x-start.x/100)))
        if (end.x - start.x == 0):
            y = ((end.y-start.y)/(end.x-start.x + 0.00000000001))*(x - start.x)+ start.y
        else:
            y = ((end.y-start.y)/(end.x-start.x ))*(x - start.x)+ start.y
        for i in range(len(x)):
            if self.binaryImage[int(y[i]), int(x[i])] == 0:
                return False
        return True