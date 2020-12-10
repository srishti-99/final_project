#!/usr/bin/python3

import math
MAX = 20

class TreeNode:

    def __init__(self, point, parent=None):
        self.children = []
        self.point = point
        self.parent = parent

    def is_root(self):
        return not self.parent

    def has_any_children(self):
        return True if self.children else False

class Tree:

    def __init__(self, root):
        self.root = root
        self.size = 0

    def add_node(self, parent, point):
        node = TreeNode(point, parent)
        parent.children.append(node)
        self.size += 1
        return node

    def get_closest_node(self, point):
        min_distance = MAX
        q = []
        q.append(self.root)
        while q:
            current_node = q.pop(0)
            current_point = current_node.point
            distance = math.sqrt(math.pow(point.x - current_point.x, 2) + math.pow(point.y - current_point.y, 2))
            if distance < min_distance:
                min_distance = distance
                closest_node = current_node
            if current_node.has_any_children():
                for child in current_node.children:
                    q.append(child)
        return closest_node
