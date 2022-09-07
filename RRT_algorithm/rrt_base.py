import cv2
import numpy as np
import math 
import random
class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x, y):
        self.x = x 
        self.y = y
        self.parent_x = []
        self.parent_y = []
class RRT():
    '''
        Class to use the RRT algorithm
    '''
    def __init__(self, image, start, goal, stepSize = 10):
        self.image = image
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ( _ , self.binaryImage) = cv2.threshold(self.grayImage, 250, 255, cv2.THRESH_BINARY)
        self.height, self.width, _ = self.image.shape
        self.start = start
        self.goal = goal
        self.stepSize = stepSize

        self.vertices = [0]
        self.vertices[0] = Nodes(self.start[0], self.start[1])
        self.vertices[0].parent_x.append(start[0])
        self.vertices[0].parent_y.append(start[1])
        self.graph = []
        self.path = []
    def planning(self):
        i = 1
        pathFound = False
        while pathFound == False:
            nx, ny = self.generatePoints()
            
            nearest_index = self.nearestNode(nx, ny)
            nearest_x = self.vertices[nearest_index].x
            nearest_y = self.vertices[nearest_index].y

            # check direct connection
            tx , ty, directCon, nodeCon = self.checkCollision(nx, ny, nearest_x, nearest_y)
            if directCon and nodeCon:
                self.vertices.append(i)
                self.vertices[i] = Nodes(tx, ty)
                self.vertices[i].parent_x = self.vertices[nearest_index].parent_x.copy()
                self.vertices[i].parent_y = self.vertices[nearest_index].parent_y.copy()

                self.vertices[i].parent_x.append(tx)
                self.vertices[i].parent_y.append(ty)

                # add to graph
                self.graph.append([(int(tx), int(ty)), (int(self.vertices[nearest_index].x), int(self.vertices[nearest_index].y))])
                # Create path
                for idx in range(len(self.vertices[i].parent_x)):
                    self.path.append((int(self.vertices[i].parent_x[idx]), int(self.vertices[i].parent_y[idx])))
                self.path.append((int(self.goal[0]), int(self.goal[1])))
                break
            elif nodeCon:
                self.vertices.append(i)
                self.vertices[i] = Nodes(tx, ty)
                self.vertices[i].parent_x = self.vertices[nearest_index].parent_x.copy()
                self.vertices[i].parent_y = self.vertices[nearest_index].parent_y.copy()
                
                self.vertices[i].parent_x.append(tx)
                self.vertices[i].parent_y.append(ty)
                
                # add to graph
                self.graph.append([(int(tx), int(ty)), (int(self.vertices[nearest_index].x), int(self.vertices[nearest_index].y))])

                i += 1
                cv2.waitKey(1)
                continue
            else:
                continue
                
    # check collision
    def isCollision(self, x1, y1, x2, y2):
        if x1 == x2 and y1 != y2:
            coord = [y1, y2]
            if y1 > y2:
                coord = [y2, y1]
            y = np.arange(coord[0], coord[1]+1, (coord[1] - coord[0])/1000)
            x = x1*np.ones(y.size)
            for i in range(x.size):
                if self.binaryImage[int(round(y[i], 0)), int(round(x[i], 0))] == 0:
                    return True
        elif x1 != x2 and y1 == y2:
            coord = [x1, x2]
            if x1 > x2:
                coord = [x2, x1]
            x = np.arange(coord[0], coord[1]+1, (coord[1] - coord[0])/1000)
            y = y1*np.ones(x.size)
            for i in range(x.size):
                if self.binaryImage[int(round(y[i], 0)), int(round(x[i], 0))] == 0:
                    return True
        elif x1 == x2 and y1 == y2:
            if self.binaryImage[int(round(y1,0)), int(round(x1,0))] == 0:
                return True
        else:
            if x1 < x2:
                x = np.arange(x1, x2+1, (x2 - x1)/1000)
            else: x = np.arange(x2, x1+1, (x1 - x2)/1000)
            y = ((y2-y1)/(x2-x1 ))*(x - x1)+ y1
            for i in range(x.size):
                if y[i] < self.height and x[i] < self.width:
                    if self.binaryImage[int(round(y[i], 0)), int(round(x[i], 0))] == 0:
                        return True
            return False
        
    # check the collision with obstacle and trim
    def checkCollision(self, x1, y1, x2, y2):
        theta =  self.getAngle(x2, y2, x1, y1)
        x =  x2 + self.stepSize*np.cos(theta)
        y =  y2 + self.stepSize*np.sin(theta)
        
        if (y < 0 or y > self.height or x < 0 or x > self.width):
            directionCon = False
            nodeCon = True
        else:
            # check direction connection
            if self.isCollision(x, y, self.goal[0], self.goal[1]):
                directionCon = False
            else:
                directionCon = True
            if self.isCollision(x, y, x2, y2):
                nodeCon = False
            else:
                nodeCon = True

        return (x, y, directionCon, nodeCon)
    # return the nearest node index
    def nearestNode(self, x, y):
        temp_dist = []
        for i in range(len(self.vertices)):
            dist =  self.getDistance(x, y, self.vertices[i].x, self.vertices[i].y)
            temp_dist.append(dist)
        
        return temp_dist.index(min(temp_dist))
    # generate a random point in the image space
    def generatePoints(self):
        new_y = random.randint(0, self.height)
        new_x = random.randint(0, self.width)

        return (new_x, new_y)
    
    @staticmethod
    def getDistance(x1,y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)
    @staticmethod
    def getAngle(x1,y1, x2, y2):
        return math.atan2(y2 - y1, x2 - y1) 

def main():
    imagePath = "/home/leanhchien/catkin_ws/src/ros_autonomous_slam/media/my_map.png"
    start = (100, 120)
    goal = (326, 279)
    image = cv2.imread(imagePath) 
    rrt = RRT(image, start, goal)

    rrt.planning()
    cv2.circle(image, start, 3, (0, 255, 0), thickness= 3, lineType= 8)
    cv2.circle(image, goal, 3, (255, 0, 0), thickness= 3, lineType= 8)
    for i in range(len(rrt.graph)):
        cv2.line(image, rrt.graph[i][0], rrt.graph[i][1], (0, 128, 128), thickness= 1, lineType=8)
    for i in range(len(rrt.path)-1):
        cv2.line(image, rrt.path[i], rrt.path[i+1], (0, 0, 255), thickness= 1, lineType=8)
    cv2.imshow("Image result", image)
    cv2.waitKey()

if __name__ == '__main__':
    main()