
from xml.dom.expatbuilder import parseString
import cv2
import numpy as np
import math 
import random
import os
import argparse

class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x, y):
        self.x = x 
        self.y = y
        self.parent_x = []
        self.parent_y = []
class RRT_algorithm:
    """Class to use the RRT algorithm"""
    node_list = [0]
    coordinates = []
    def __init__(self, image1, image2, start = (20, 20), end = (450, 250), stepSize = 10):
        self.img = image1
        self.img2 = image2
        self.start = start
        self.end = end
        self.stepSize = stepSize
        # node _list =  [0] # list to store all the node points
        self.node_list[0] = Nodes(self.start[0], self.start[1])
        self.node_list[0].parent_x.append(start[0])
        self.node_list[0].parent_y.append(start[1])
    # check collision
    def collision(self, x1, y1, x2, y2):
        color = []
        x = list(np.arange(x1, x2, (x2-x1)/100))
        y = list(((y2-y1)/(x2-x1))*(x-x1)+y1)
        print("Collision ", x, y)
        for i in range(len(x)):
            print(int(x[i],), int(y[i]))
            color.append(self.img[int(y[i]), int(x[i])])
        if (0 in color):
            return True # collision
        else:
            return False # no collision
    # check the collision with obstacle and trim
    def check_collision(self, x1, y1, x2, y2):
        _, theta = self.dist_and_angle(x2, y2, x1, y1)
        x = x2 + self.stepSize*np.cos(theta)
        y = y2 + self.stepSize*np.sin(theta)
        print(x2, y2, x1, y1)
        print("Theta: ", theta)
        print("Check Collision", x, y)
        # TODO: trim the branch if its going out of image area
        # print("Image shape:", self.img.shape)
        hy, hx = self.img.shape

        if (y < 0 or y > hy or x <0 or x > hx):
            print("Point out of image bound")
            directCon = False
            nodeCon = False
        else:
            # check direction connection
            if self.collision(x, y, self.end[0], self.end[1]):
                directCon = False
            else :
                directCon = True
            # check connection between two nodes
            if self.collision(x, y, x2, y2):
                nodeCon = False
            else:
                nodeCon = True
        return (x, y, directCon, nodeCon)    

    # return dist and angle between new point and nearest node    
    def dist_and_angle(self, x1, y1, x2, y2):
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        angle = math.atan2(y2-y1, x2-x1)
        return (dist, angle)

    # return the nearest node  index
    def nearestNode(self, x, y):
        temp_dist = []
        for i in range(len(self.node_list)):
            dist, _ = self.dist_and_angle(x, y, self.node_list[i].x, self.node_list[i].y)
            temp_dist.append(dist)
        return temp_dist.index(min(temp_dist))
    
    # generate a random  point  in the image space
    def rnd_point(self, h, l):
        new_y = random.randint(0, h)
        new_x = random.randint(0, l)

        return (new_x, new_y)
    
    # RRT algorithm
    def RRT(self):
        h, l = self.img.shape # dim of the loaded image

        print("Image shape = ", self.img.shape)

        cv2.circle(self.img2, self.start, 5, (0, 0, 255), thickness= 3, lineType= 8)
        cv2.circle(self.img2, self.end, 5, (0, 0, 255), thickness= 3, lineType= 8)

        i = 1
        pathFound = False
        while pathFound == False:
            nx, ny = self.rnd_point(h, l)
            print("Random points: ", nx, ny)
            
            nearest_ind = self.nearestNode(nx, ny)
            nearest_x = self.node_list[nearest_ind].x
            nearest_y = self.node_list[nearest_ind].y
            print("Nearest node coordinate: ", nearest_x, nearest_y)

            # check direct connection
            tx, ty, directCon, nodeCon = self.check_collision(nx, ny, nearest_x, nearest_y)
            print("Check collision: ", tx, ty, directCon, nodeCon)

            if directCon and  nodeCon:
                print("Node can connect directly with end")
                self.node_list.append(i)
                self.node_list[i] = Nodes(tx, ty)
                self.node_list[i].parent_x = self.node_list[nearest_ind].parent_x.copy()
                self.node_list[i].parent_y = self.node_list[nearest_ind].parent_y.copy()
                self.node_list[i].parent_x.append(tx)
                self.node_list[i].parent_y.append(ty)

                cv2.circle(self.img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness= 3, lineType= 8)
                cv2.line(self.img2, (int(tx), int(ty)), (int(self.node_list[nearest_ind].x), int(self.node_list[nearest_ind].y)), (0, 255, 0), thickness=  1, lineType= 8)
                cv2.line(self.img2, (int(tx), int(ty)), self.end, (255, 0, 0), thickness= 2, lineType= 8)

                print("Path has been found")
                print("Parent_x: ",self.node_list[i].parent_x)

                for j  in range(len(self.node_list[i].parent_x)-1):
                    cv2.line(self.img2, (int(self.node_list[i].parent_x[j]), int(self.node_list[i].parent_y[j])),(int(self.node_list[i].parent_x[j+1]), int(self.node_list[i].parent_y[j+1])), (255, 0, 0), thickness= 2, lineType= 8)
                # cv2.waitKey(1)
                cv2.imwrite("media/"+ str(i) + ".jpg", self.img2)
                cv2.imwrite("out.jpg", self.img2)

                break
            elif nodeCon:
                print("Node connected")
                self.node_list.append(i)
                self.node_list[i] = Nodes(tx, ty)
                self.node_list[i].parent_x = self.node_list[nearest_ind].parent_x.copy()
                self.node_list[i].parent_y = self.node_list[nearest_ind].parent_y.copy()

                self.node_list[i].parent_x.append(tx)
                self.node_list[i].parent_y.append(ty)

                i = i + 1

                # display
                cv2.circle(self.img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness=3, lineType=8)
                cv2.line(self.img2, (int(tx), int(ty)), (int(self.node_list[nearest_ind].x), int(self.node_list[nearest_ind].y)), (0, 255, 0), thickness=1, lineType=8)
                cv2.imwrite("media/"+ str(i) + ".jpg", self.img2)
                cv2.imshow("SDC", self.img2)
                cv2.waitKey(1)
                continue
            else:
                print("No direct con. and no node con.:(Generate new rnd numbers)")
                continue
    def drawCircle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.circle(self.img2, x, y, 5, (255, 0, 0), -1)
            self.coordinates.append(x)
            self.coordinates.append(y)

def main():
    
    parser = argparse.ArgumentParser(description= "Below are the params")
    parser.add_argument('-selectPoint', help="Select start and end points from figure", action= 'store_true')

    args = parser.parse_args()
    dir = "media"
    for f in os.listdir(dir):
        os.remove(os.path.join(dir, f))
    imagePath = "world4.png"
    image1 = cv2.imread(imagePath, 0) # load grayscale maze image
    image2 = cv2.imread(imagePath) #load colored maze image
    RRT_object= RRT_algorithm(image1, image2, start= (25, 30), end=(450, 300))

    if args.selectPoint:
        print("Select start and end points by double clicking, press 'escape' to exit")
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', RRT_object.drawCircle)
        while (1):
            cv2.imshow('image', image2)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                break
            RRT_object.start = (RRT_object.coordinates[0], RRT_object.coordinates[1])
            RRT_object.end = (RRT_object.coordinates[2], RRT_object.coordinates[3])
    # run the RRT algorithm
    RRT_object.RRT()
if __name__ == '__main__':
    main()
       
    