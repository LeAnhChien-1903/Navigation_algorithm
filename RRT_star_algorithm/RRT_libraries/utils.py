"""
    utils for collision check
"""
import math
import numpy as np
import os
import sys

sys.path.insert(0, 'RRT_libraries')
import env
from RRT import Node

class Utlis:
    def __init__(self):
        self.env = env.Env()
        self.delta = 0.5
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
    
    def update_obs(self, obs_cir, obs_bound, obs_rect):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rect
    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []
        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)
        
        return obs_list