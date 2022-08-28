import math
import sys
sys.path.insert(0, 'RRT_libraries')
from node import Node

class Cost:
    @staticmethod
    def getDistance(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
    @staticmethod
    def getAngle(start, end):
        return math.atan2(end.y - start.y, end.x - start.x)
    def getCostOfPath(self, path):
        cost = 0
        for i in range(len(path)-1):
            start = Node(path[i])
            end = Node(path[i+1])
            distance = self.getDistance(start, end)
            cost += distance
        
        return cost
