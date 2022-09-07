import numpy as np
import cv2 
import math
import random
class Node:
    def __init__(self, coordinate):
        self.x = coordinate[0]
        self.y = coordinate[1]
        self.parent = None

class RRT:
    def __init__(self, image, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.image = image
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ( _ , self.binaryImage) = cv2.threshold(self.grayImage, 250, 255, cv2.THRESH_BINARY)
        self.height, self.width, _ = self.image.shape
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertices = [self.s_start]
        self.path = []
    
    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertices,node_rand)
            node_new = self.new_state(node_near, node_rand)
            if i% 100 == 0:
                print(i)
            if node_new and not self.isCollision(node_near, node_rand):
                self.vertices.append(node_new)
                dist =  self.getDistance(node_new, self.s_goal)

                if dist <= self.step_len and not self.isCollision(node_new, node_near):
                    self.new_state(node_new,self.s_goal)
                    self.path = self.extract_path(node_new)
                    break
    
    def generate_random_node(self, goal_sample_rate):
        if random.random() > goal_sample_rate:
            return Node((random.randint(1, self.width - 1),
                        random.randint(1, self.height - 1)))
        return self.s_goal
    def nearest_neighbor(self,node_list, n):
        temp_dist = []
        for i in range(len(node_list)):
            dist = self.getDistance(node_list[i], n)
            temp_dist.append(dist)
        return node_list[temp_dist.index(min(temp_dist))]
    @staticmethod
    def getDistance(node_start, node_goal):
        return math.hypot(node_goal.x - node_start.x, node_goal.y - node_start.y)
    @staticmethod
    def getAngle(node_start, node_goal):
        return math.atan2(node_goal.y - node_start.y, node_goal.x - node_start.y) 
    def new_state(self, node_start, node_goal):
        dist = self.getDistance(node_start,node_goal)
        theta = self.getAngle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                        node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start
        return node_new

    def extract_path(self, node_end):
        path = [[int(self.s_goal.x), int(self.s_goal.y)]]
        node = node_end

        while node.parent is not None:
            path.append([int(node.x), int(node.y)])
            node = node.parent
        path.append([int(node.x), int(node.y)])
        path.reverse()
        return path
    def isCollision(self, nodeStart, nodeEnd):
        if nodeStart.x == nodeEnd.x and nodeStart.y != nodeEnd.y:
            coord = [nodeStart.y, nodeEnd.y]
            if nodeStart.y > nodeEnd.y:
                coord = [nodeEnd.y, nodeStart.y]
            y = np.arange(coord[0], coord[1]+1, (coord[1] - coord[0])/1000)
            x = nodeStart.x*np.ones(y.size)
            for i in range(x.size):
                if self.binaryImage[int(round(y[i], 0)), int(round(x[i], 0))] == 0:
                    return True
        elif nodeStart.x != nodeEnd.x and nodeStart.y == nodeEnd.y:
            coord = [nodeStart.x, nodeEnd.x]
            if nodeStart.x > nodeEnd.x:
                coord = [nodeEnd.x, nodeStart.x]
            x = np.arange(coord[0], coord[1]+1, (coord[1] - coord[0])/1000)
            y = nodeStart.y*np.ones(x.size)
            for i in range(x.size):
                if self.binaryImage[int(round(y[i], 0)), int(round(x[i], 0))] == 0:
                    return True
        elif nodeStart.x == nodeEnd.x and nodeStart.y == nodeEnd.y:
            if self.binaryImage[int(round(nodeStart.y,0)), int(round(nodeStart.x,0))] == 0:
                return True
        else:
            if nodeStart.x < nodeEnd.x:
                x = np.arange(nodeStart.x, nodeEnd.x+1, (nodeEnd.x - nodeStart.x)/1000)
            else: x = np.arange(nodeEnd.x, nodeStart.x+1, (nodeStart.x - nodeEnd.x)/1000)
            y = ((nodeEnd.y-nodeStart.y)/(nodeEnd.x-nodeStart.x ))*(x - nodeStart.x)+ nodeStart.y
            for i in range(x.size):
                if y[i] < self.height and x[i] < self.width:
                    if self.binaryImage[int(round(y[i], 0)), int(round(x[i], 0))] == 0:
                        return True
            return False

def main():
    x_start =  [100, 50]
    x_goal = [400, 300]
    x_start1 = [100, 120]
    x_goal1 = [326, 279]
    step_len = 100
    goal_sample_rate = 0.05
    numOfNodes = 10000
    imagePath1 = "/home/leanhchien/catkin_ws/src/ros_autonomous_slam/media/my_map.png"
    imagePath = "/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"

    image = cv2.imread(imagePath1)                      
    rrt = RRT(image, x_start1, x_goal1, step_len, goal_sample_rate, numOfNodes)
    rrt.planning()
    cv2.circle(image, (rrt.s_start.x, rrt.s_start.y), 5, (255, 0, 0), thickness= 2, lineType= 8)
    cv2.circle(image, (rrt.s_goal.x, rrt.s_goal.y), 5, (0, 255, 0), thickness= 2, lineType= 8)
    for i in range(1,  len(rrt.vertices)):
        cv2.line(image, (int(rrt.vertices[i].x),int(rrt.vertices[i].y)), (int(rrt.vertices[i].parent.x),int(rrt.vertices[i].parent.y)), (0, 128, 128), thickness = 1, lineType = 8)
    for i in range(len(rrt.path)-1):
       cv2.line(image, tuple(rrt.path[i]),tuple(rrt.path[i+1]), (0, 0, 255), thickness= 2, lineType=8)
    cv2.imshow("Image", image)
    cv2.imshow("Binary Image", rrt.binaryImage)
    cv2.waitKey()
if __name__ == '__main__':
    main()     