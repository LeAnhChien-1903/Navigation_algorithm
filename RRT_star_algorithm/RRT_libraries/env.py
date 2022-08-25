"""
    Environment for rrt_2D
"""
class Env:
    def __init__(self, x_range = (0, 50), y_range = (0, 30)):
        self.x_range = x_range
        self.y_range = y_range
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()
    def obs_boundary(self):
        obs_boundary = [
            [0, 0, 1, self.y_range[1]],
            [0, self.y_range[1], self.x_range[1], 1],
            [1, 0, self.x_range[1], 1],
            [self.x_range[1], 1, 1, self.y_range[1]]
        ]
        return obs_boundary
    @staticmethod
    def obs_circle():
        obs_circle = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]
        return obs_circle
    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle
