from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from Node import Node
from Vessel import Vessel
import numpy as np
    

class Tree:
    def __init__(self, root):
        if root is None:
            raise ValueError("Root node must be defined.")
        self.root = root
        self.vessels = []
        self.nodes = [root]
        self.medium_nodes = []
        self.kd_tree = KDTree()
        self.collision_radius = 0.01  # Defina um raio de colisão mínimo

    def add_terminal_node(self, x, y, flow=1.0):
        nearest_node = self.find_nearest_node(x, y)
        
        if nearest_node is None:
            print("No valid parent node found for the new terminal node. Skipping node.")
            return None

        valid_parent_node = self.find_valid_parent_node(x, y, nearest_node)
        if valid_parent_node is None:
            print(f"No valid parent node found for the new terminal node at ({x}, {y}). Node not added.")
            return None

        if self.check_collision(x, y, valid_parent_node):
            print(f"Collision detected at ({x}, {y}). Node not added.")
            return None

        new_node = Node(x, y, flow, valid_parent_node)
        new_vessel = Vessel(valid_parent_node, new_node)

        self.vessels.append(new_vessel)
        self.nodes.append(new_node)
        self.kd_tree = KDTree([(node.x, node.y) for node in self.nodes])

        self.local_optimization(new_node)

        print(f"Node at ({x}, {y}) added successfully.")
        return new_node

    def find_nearest_node(self, x, y):
        if self.kd_tree.size == 0:
            return None
        distance, index = self.kd_tree.query((x, y))
        return self.nodes[index]

    def find_valid_parent_node(self, x, y, initial_node):
        if self.kd_tree.size == 0:
            return None
        distances, indices = self.kd_tree.query((x, y), k=len(self.medium_nodes))
        if isinstance(indices, np.int64):  # Verificar se indices é um único valor
            indices = [indices]  # Converter para lista
        for index in indices:
            potential_parent = self.medium_nodes[index]
            if len(potential_parent.children) < 2 and not self.check_collision(x, y, potential_parent):
                return potential_parent
        return None
    
    def medium_node_in_vessel(self, xa, xb, ya, yb):
        return (xa + xb) / 2, (ya + yb) / 2

    def check_collision(self, x, y, parent_node):
        for node in self.nodes:
            if np.linalg.norm(node.position() - np.array([x, y])) < self.collision_radius:
                return True

        for vessel in self.vessels:
            if self.do_edges_intersect(parent_node.position(), np.array([x, y]), vessel.parent.position(), vessel.child.position()):
                return True

        return False

    def do_edges_intersect(self, p1, p2, q1, q2):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)

    def local_optimization(self, new_node):
        def optimization_function(position):
            original_position = new_node.position().copy()
            new_node.x, new_node.y = position
            total_cost = self.compute_total_cost()
            new_node.x, new_node.y = original_position
            return total_cost

        result = minimize(optimization_function, new_node.position(),
                          bounds=[(0, 1), (0, 1)])
        optimized_position = result.x
        new_node.x, new_node.y = optimized_position

    def compute_total_cost(self, gamma=2.7):
        total_cost = 0
        for vessel in self.vessels:
            cost = (vessel.length) * (vessel.diameter ** gamma)
            total_cost += cost
        return total_cost

    def plot_tree(self):
        plt.figure(figsize=(12, 12))
        for vessel in self.vessels:
            x_values = [vessel.parent.x, vessel.child.x]
            y_values = [vessel.parent.y, vessel.child.y]
            plt.plot(x_values, y_values, color='blue', linewidth=vessel.diameter * 2)
        plt.scatter([node.x for node in self.nodes], [node.y for node in self.nodes], color='red', s=10)
        plt.xlim(0, 1)  # Ajuste para centralizar e dar zoom
        plt.ylim(0, 1)  # Ajuste para centralizar e dar zoom
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
