from dis import dis
import math

class Cost:
    @staticmethod
    def get_distance_and_angle(x, y):
        dx = y[0] - x[0]
        dy = y[1] - x[1]
        return math.hypot(dx, dy), math.atan2(dy, dx)
    def getCostOfPath(self, path):
        cost = 0
        for i in range(len(path)-1):
            distance, _ = self.get_distance_and_angle(path[i], path[i+1])
            cost += distance
        
        return cost
