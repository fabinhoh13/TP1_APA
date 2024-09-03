import numpy as np

class Node:
    def __init__(self, x, y, flow, parent=None):
        self.x = x
        self.y = y
        self.flow = flow
        self.parent = parent
        self.children = []
        self.expected_children = 0  # New variable to track the expected number of children
        if parent:
            parent.children.append(self)
            print(f"Nó filho na posição ({self.x}, {self.y}) conectado ao nó pai na posição ({parent.x}, {parent.y})")
        else:
            print(f"Nó raiz na posição ({self.x}, {self.y}) criado")

    def position(self):
        return np.array([self.x, self.y])
