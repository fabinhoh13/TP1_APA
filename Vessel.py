import numpy as np

class Vessel:
    def __init__(self, parent, child):
        if parent is None or child is None:
            raise ValueError("Parent and child nodes must be defined.")
        self.parent = parent
        self.child = child
        self.length = self.compute_length()
        self.diameter = self.compute_diameter()
        self.medium_point_y = (parent.y + child.y) / 2
        self.medium_point_x = (parent.x + child.x) / 2
        self.medium_point_flow = child.flow  # Usar o fluxo do filho para calcular o ponto médio
        self.usedMedium = False
    
    def get_medium_point(self):
        return (self.medium_point_x, self.medium_point_y)

    def compute_length(self):
        return np.linalg.norm(self.parent.position() - self.child.position())

    def compute_diameter(self):
        if not self.parent.children:
            return 1.0  # Diâmetro inicial padrão
        return max(0.1, (self.child.flow ** (1/3)))  # Ajuste para evitar diâmetros muito pequenos
