import cv2
import sys

sys.path.insert(0, 'RRT_libraries')
import env

class PlotVer2:
    def __init__(self, image, x_start, x_goal):
        self.image = image
        self.x_start = x_start
        self.x_goal = x_goal
    def animation(self, nodelist, path, animation = False):
        self.plot_visited(nodelist, animation)
        self.plot_path(path)
    def plot_visited(self, nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    cv2.line(self.image, (node.parent.x, node.x), (node.parent.y, node.y), (0, 255, 0), thickness=1)
        else:
            for node in nodelist:
                if node.parent:
                    cv2.line(self.image, (node.parent.x, node.x), (node.parent.y, node.y), (0, 255, 0), thickness=1)
    def plot_path(self, path):
        if len(path) != 0:
            for i in range(len(path)-1):
                cv2.line(self.image, (path[i][0], path[i][1]), (path[i+1][0], path[i+1][1]), (0, 0, 255), thickness=1)       
