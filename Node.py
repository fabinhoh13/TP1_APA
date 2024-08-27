import numpy as np

class Node:
    def __init__(self, x, y, flow, parent=None):
        self.x = x
        self.y = y
        self.flow = flow
        self.parent = parent
        self.children = []
        if parent:
            parent.children.append(self)

    def position(self):
        return np.array([self.x, self.y])