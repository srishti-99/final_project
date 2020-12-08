import pygame
import math

class HybridState:

    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.bounding_box = []

    def check_collision(self, generated_point):
        
