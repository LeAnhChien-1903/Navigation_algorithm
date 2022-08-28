import sys
sys.path.insert(0, '/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in ubuntu
sys.path.insert(0, 'D:/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in windows
import cv2
from cost import Cost
from node import Node
import plottingVer2
import numpy as np

a = np.arange(0, 10, 0.1)
y = a + 1

print(a, y)
