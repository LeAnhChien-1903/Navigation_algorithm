"""
    RRT_star_2D for image
"""
from turtle import heading
import cv2
import sys
import math
import numpy as np
import random

sys.path.insert(0, '/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in ubuntu
sys.path.insert(0, 'D:/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in windows
import plottingVer2
import queue
import utilsVer2
from node import Node
from cost import Cost

class RRTStar:
    def __init__(self, image, x_start, x_goal, step_len, goal_sample_rate, search_radius, iter_max):
        self.image = image
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        (_, self.binaryImage) = cv2.threshold(self.grayImage, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        self.height, self.width, _ = self.image.shape
        self.s_start = x_start
        self.s_goal = x_goal
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []

        self.utils = utilsVer2.Utils(self.binaryImage)

    def planning(self):
        for k in range(self.iter_max):
            pass

    def new_states(self, node_start, node_goal):
        dist = Cost.getDistance(node_start, node_goal)
        theta = Cost.getAngle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + int(dist * math.cos(theta)), node_start.y + int(dist * math.sin(theta))))
        
        node_new.parent = node_start
    
    def choose_parent(self, node_new, neighbor_index):
        cost = [self.getNewCost(self.vertex[i], node_new) for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]
    
    def getNewCost(self, node_start, node_end):
        dist = Cost.getDistance(node_start, node_end)
        
        return self.cost(node_start) + dist

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]
            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def searchGoalParent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len] 
        
        if len(node_index) > 0:
            cost_list = [dist_list[i] +  self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.checkCollision(self.vertex[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]
        
        return len(self.vertex) - 1
    def generateRandomNode(self, goal_sample_rate):
        if np.random.random() > goal_sample_rate:
            while True:
                x = random.randint(0, self.width)
                y = random.randint(0, self.height)
                if self.binaryImage[y, x] == 255:
                    return Node(x, y)
        return self.s_goal
    
    def findNearNeighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius)
        
        distTable = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        distTableIndex = [ind for ind in range(len(distTable)) if distTable[ind] <= r and 
                          not self.utils.checkCollision(node_new, self.vertex[ind])]
    
    @staticmethod
    def nearestNeighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def updateCost(self, parentNode):
        OPEN = 