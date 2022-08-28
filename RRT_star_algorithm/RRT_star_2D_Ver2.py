"""
    RRT_star_2D for image
"""

from ast import NodeTransformer
import sys
import math
from unittest import result
import numpy as np
import random
import cv2

sys.path.insert(0, '/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in ubuntu
sys.path.insert(0, 'D:/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in windows
import plottingVer2
import queue
import collision
from node import Node

class RRTStar:
    def __init__(self, image, x_start, x_goal, step_len, goal_sample_rate, search_radius, iter_max):
        self.image = image
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ( _ , self.binaryImage) = cv2.threshold(self.grayImage, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        self.height, self.width, _ = self.image.shape
        self.result = np.copy(image)
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertices = [self.s_start]
        self.edges = []
        self.path = []

        self.plotting = plottingVer2.PlotVer2(self.image, self.s_start, self.s_goal)
        self.collision = collision.Collision(self.binaryImage)
    def planning(self):
        for iter in range(self.iter_max):
            nodeRand = self.SampleFree()
            nodeNearest = self.Nearest(nodeRand)
            nodeNew = self.Steer(nodeNearest, nodeRand)
            if self.collision.CollisionFree(nodeNearest, nodeNew):
                listOfNodeNear, _  = self.Near(nodeNew)
                self.vertices.append(nodeNew)
                nodeMin = nodeNearest
                costMin = self.Cost(nodeNearest) + self.costOfLine(nodeNearest, nodeNew)
                for nodeNear in listOfNodeNear :
                    if self.collision.CollisionFree(nodeNear, nodeNew) and self.Cost(nodeNear) + self.costOfLine(nodeNear, nodeNew) < costMin:
                        nodeMin = nodeNear
                        costMin = self.Cost(nodeNear) + self.costOfLine(nodeNear, nodeNew)
                self.edges.append([nodeMin, nodeNew])
                for nodeNear in listOfNodeNear:
                    if self.collision.CollisionFree(nodeNew, nodeNear) and self.Cost(nodeNew) + self.costOfLine(nodeNew, nodeNear) < self.Cost(nodeNear):
                        nodeParent = self.Parent(nodeNear)
                        self.edges.remove([nodeParent, nodeNear])
                        self.edges.append([nodeNew, nodeNear])
        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertices[index])
        #print(self.path[len(self.path)-1])
        self.plotting.plot_path(self.path)  
        result = self.plotting.image
        cv2.imshow("Result", result)
        cv2.waitKey() 

    def SampleFree(self):
        while True:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            if self.binaryImage[y, x] == 255:
                return Node([x, y])
    def Nearest(self, node):
        return self.vertices[int(np.argmin([math.hypot(nd.x - node.x, nd.y - node.y) for nd in self.vertices]))]
    def Steer(self, nodeStart, nodeEnd):
        dist = self.getDistance(nodeStart, nodeEnd)
        theta = self.getAngle(nodeStart, nodeEnd)

        dist = min(self.step_len, dist)
        node_new = Node((nodeStart.x + int(dist * math.cos(theta)),
                         nodeStart.y + int(dist * math.sin(theta))))

        node_new.parent = nodeStart

        return node_new
    def Near(self, nodeNew):
        n = len(self.vertices) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - nodeNew.x, nd.y - nodeNew.y) for nd in self.vertices]
        nodeNearIndex = [ ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.collision.CollisionFree(nodeNew, self.vertices[ind])]

        listOfNodeNear = [self.vertices[ind] for ind in nodeNearIndex]

        return listOfNodeNear, nodeNearIndex
    def Cost(self, node_p):
        node = node_p
        cost = 0.0

        while node.parent != None:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost
    def costOfLine(self, nodeStart, nodeEnd):
        return math.hypot(nodeEnd.x - nodeStart.x, nodeEnd.y - nodeStart.y)
    def Parent(self, node):
        _ , indexNearNode = self.Near(node) 
        cost = [self.getNewCost(self.vertices[i], node) for i in indexNearNode]
        
        cost_min_index = indexNearNode[int(np.argmin(cost))]

        node.parent = self.vertices[cost_min_index]

        for i in indexNearNode:
            nodeNear = self.vertices[i]
            if self.Cost(nodeNear) > self.getNewCost(node, nodeNear):
                nodeNear.parent = node
        
    def getNewCost(self, nodeStart, nodeEnd):
        dist = self.getDistance(nodeStart, nodeEnd)
        
        return self.Cost(nodeStart) + dist
    
    @staticmethod
    def getDistance(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
    @staticmethod
    def getAngle(start, end):
        return math.atan2(end.y - start.y, end.x - start.x)
    def updateCost(self, parentNode):
        OPEN = queue.QueueFIFO()
        OPEN.put(parentNode)

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

            for node_c in node.child:
                node_c.Cost = self.getNewCost(node, node_c)
                OPEN.put(node_c)

    def extractPath(self, nodeEnd):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = nodeEnd
        while node.parent is not None:
            path.append([node.x, node.y])
        path.append([node.x, node.y])
        
        return path
    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertices]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.Cost(self.vertices[i]) for i in node_index
                         if self.collision.CollisionFree(self.vertices[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertices) - 1
def main():
    x_start = (100, 50)
    x_goal = (400, 300)


    imagePath = "D:/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"
    image = cv2.imread(imagePath)
    cv2.circle(image, x_start, 5, (0, 0, 255), thickness= 3, lineType= 8)
    cv2.circle(image, x_goal, 5, (0, 0, 255), thickness= 3, lineType= 8)
    # print(image.shape)
    # cv2.imshow("Original Image", image)
    # cv2.waitKey()
    rrt_star = RRTStar(image, x_start, x_goal, 10, 0.1, 20, 10000)
    rrt_star.planning()
    print("Final Path: ")
    for vertex in rrt_star.path:
        print("({}, {})".format(vertex[0], vertex[1]))
    # cost_obj = Cost()
    # print(cost_obj.getCostOfPath(rrt_star.path))

if __name__ == "__main__":
    main()