"""
    RRT_star_2D
"""

from importlib.resources import path
import random
import sys
import math
import numpy as np
import cv2

sys.path.insert(0, '/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in ubuntu
sys.path.insert(0, 'D:/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in windows
import queue

class Node:
    def __init__(self, coordinate):
        self.x = coordinate[0]
        self.y = coordinate[1]
        self.parent = None
class RRTStar:
    def __init__(self, image, x_start, x_goal, step_len, goal_sample_rate, search_radius, iter_max):
        self.image = image
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ( _ , self.binaryImage) = cv2.threshold(self.grayImage, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        self.height, self.width, _ = self.image.shape
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertices = [self.s_start]
        self.path = []
        self.costOfPath = 0.0
    def planning(self):
        for k in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertices, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if k % 500 == 0:
                print(k)

            if node_new and not self.isCollision(node_near, node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                cv2.circle(self.image, (int(node_new.x), int(node_new.y)), 1, (0, 120, 255), thickness= 2, lineType= 8)
                self.vertices.append(node_new)

                if neighbor_index:
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertices[index])
        self.costOfPath = self.getCostOfPath(self.path)
    
    def new_state(self, node_start, node_goal):
        dist = self.getDistance(node_start,node_goal)
        theta = self.getAngle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                        node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start
        return node_new

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertices[i], node_new) for i in neighbor_index]
        
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertices[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertices[i]
            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertices]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i]+ self.cost(self.vertices[i]) for i in node_index
                         if not self.isCollision(self.vertices[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertices) - 1

    def get_new_cost(self, node_start, node_end):
        dist = self.getDistance(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self, goal_sample_rate):
        if random.random() > goal_sample_rate:
            return Node((np.random.uniform(1, self.width - 1),
                        np.random.uniform(1, self.height - 1)))
        return self.s_goal

    def find_near_neighbor(self, node_new):
        n = len(self.vertices) + 1
        r = min((self.search_radius + math.sqrt(math.log(n)/n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertices]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and 
                            not self.isCollision(node_new, self.vertices[ind])]
        
        return dist_table_index

    def isCollision(self, nodeStart, nodeEnd):
        if nodeStart.x == nodeEnd.x and nodeStart.y != nodeEnd.y:
            coord = [nodeStart.y, nodeEnd.y]
            if nodeStart.y > nodeEnd.y:
                coord = [nodeEnd.y, nodeStart.y]
            y = np.arange(coord[0], coord[1], (coord[1] - coord[0])/100)
            x = nodeStart.x*np.ones(y.size)
            for i in range(x.size):
                if self.binaryImage[int(y[i]), int(x[i])] == 0:
                    return True
        elif nodeStart.x != nodeEnd.x and nodeStart.y == nodeEnd.y:
            coord = [nodeStart.x, nodeEnd.x]
            if nodeStart.x > nodeEnd.x:
                coord = [nodeEnd.x, nodeStart.x]
            x = np.arange(coord[0], coord[1], (coord[1] - coord[0])/100)
            y = nodeStart.y*np.ones(x.size)
            for i in range(x.size):
                if self.binaryImage[int(y[i]), int(x[i])] == 0:
                    return True
        elif nodeStart.x == nodeEnd.x and nodeStart.y == nodeEnd.y:
            if self.binaryImage[int(nodeStart.y), int(nodeStart.x)] == 0:
                return True
        else:
            if nodeStart.x < nodeEnd.x:
                x = np.arange(nodeStart.x, nodeEnd.x, (nodeEnd.x - nodeStart.x)/100)
            else: x = np.arange(nodeEnd.x, nodeStart.x, (nodeStart.x - nodeEnd.x)/100)
            y = ((nodeEnd.y-nodeStart.y)/(nodeEnd.x-nodeStart.x ))*(x - nodeStart.x)+ nodeStart.y
            for i in range(x.size):
                if y[i] < self.height and x[i] < self.width:
                    if self.binaryImage[int(y[i]), int(x[i])] == 0:
                        return True
            return False
    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]))]
    @staticmethod
    def getDistance(node_start, node_goal):
        return math.hypot(node_goal.x - node_start.x, node_goal.y - node_start.y)
    @staticmethod
    def getAngle(node_start, node_goal):
        return math.atan2(node_goal.y - node_start.y, node_goal.x - node_start.y)   
    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0
        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent
        
        return cost
    def update_cost(self, parent_node):
        OPEN = queue.QueueFIFO()
        OPEN.put(parent_node)

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

            for node_c in node.child:
                node_c.Cost = self.get_new_cost(node, node_c)
                OPEN.put(node_c)
    def extract_path(self, node_end):
        path = [[int(self.s_goal.x), int(self.s_goal.y)]]
        node = node_end

        while node.parent is not None:
            path.append([int(node.x), int(node.y)])
            node = node.parent
        path.append([int(node.x), int(node.y)])
        path.reverse()
        return path
    def getCostOfPath(self, path):
        costOfPath = 0.0
        for i in range(len(path)-1):
            costOfPath += math.hypot(path[i+1][0] - path[i][0],path[i+1][1] - path[i][1])
        
        return costOfPath

def main():
    x_start = [100, 50]
    x_goal = [400, 300]

    step_len = 10
    radius = 20
    numOfNodes = 5000
    imagePath = "D:/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"
    imageResultPath = "D:/Navigation_algorithm/RRT_star_algorithm/result/world1/image file/image_{}/{}_{}_{}.png".format(numOfNodes, 10, step_len, radius)
    image = cv2.imread(imagePath)
    rrt_star = RRTStar(image, x_start, x_goal, step_len, 0.10, radius, numOfNodes)
    rrt_star.planning()
    print(rrt_star.path)
    print(rrt_star.costOfPath)
    cv2.circle(image, x_start, 5, (0, 0, 255), thickness= 2, lineType= 8)
    cv2.circle(image, x_goal, 5, (255, 0, 0), thickness= 2, lineType= 8)
    for i in range(len(rrt_star.path)-1):
       cv2.line(image, rrt_star.path[i],rrt_star.path[i+1], (0, 255, 0), thickness= 2, lineType=8)
    cv2.imwrite(imageResultPath, image)
    cv2.imshow("Image", image)
    cv2.waitKey()

if __name__ == '__main__':
    main()