import sys
from turtle import width
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from random import randint, random
sys.path.insert(0, '/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in ubuntu
sys.path.insert(0, 'D:/Navigation_algorithm/RRT_star_algorithm/RRT_libraries') # path directory in windows
import env

class GenerateMap:
    def __init__(self, order):
        self.order = order
        self.imagePath = "D:/Navigation_algorithm/RRT_star_algorithm/RRT_map/world{}.png".format(self.order)
        self.env = env.Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
    def plot_grid(self):
        fig, ax = plt.subplots()
        
        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle( 
                    (ox, oy), w, h,
                    edgecolor = 'black',
                    facecolor = 'black',
                    fill = True
                )
            )
        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor = 'black',
                    facecolor = 'gray',
                    fill =  True
                )
            )
        for (ox, oy, r) in self.obs_circle:
            ax.add_patch(
               patches.Circle(
                (ox, oy), r,
                edgecolor = 'black',
                facecolor = 'gray',
                fill = True
               )
            )
        plt.plot()
        ax.set_axis_off()
        fig.add_axes(ax)
        plt.savefig(self.imagePath, dpi = 100)
        fig.show()
    def updateWorld(self, order_new = 1):
        self.order = order_new
        self.imagePath = "D:/Navigation_algorithm/RRT_star_algorithm/RRT_map/world{}.png".format(self.order)
        for i in range(len(self.obs_circle)):
            radiusCircle = randint(2, 5)
            x_radius = randint(radiusCircle + 2 , self.env.x_range[1] - radiusCircle - 2)
            y_radius = randint(radiusCircle + 2 , self.env.y_range[1] - radiusCircle - 2)
            self.obs_circle[i] = [x_radius, y_radius, radiusCircle]
        for i in range(len(self.obs_rectangle)):
            height = randint(3, 10)
            width = randint(3, 10)
            x = randint(height + 2 , self.env.x_range[1] - height - 2)
            y = randint(width + 2 , self.env.y_range[1] - width - 2)
            self.obs_rectangle[i] = [x, y, height, width]

    def generateImage(self):
        image = cv2.imread(self.imagePath)
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        start = []
        end = []
        height, width, _ = image.shape
        flag = False
        for x in range(int(width/2)):
            for y in range(int(height/2)):
                if grayImage[y, x] == 0: 
                    start.append(x)
                    start.append(y)
                    flag = True
                    break
            if flag == True: break
        flag = False
        for x in range(width-1, int(width/2), -1):
            for y in range(height-1, int(height/2), -1):
                if grayImage[y, x] == 0:
                    end.append(x)
                    end.append(y)
                    flag = True
                    break
            if flag == True: break
        
        image = image[start[1]:end[1], start[0]:end[0]]

        cv2.imwrite(self.imagePath, image)

def main():
    map = GenerateMap(1)
    map.updateWorld(order_new=9)
    map.plot_grid()
    map.generateImage()

    
if __name__ =='__main__':
    main()