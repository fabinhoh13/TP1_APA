from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from Node import Node
from Vessel import Vessel
import numpy as np
from shapely.geometry import LineString, Point






class Tree:
    def __init__(self, root):
        if root is None:
            raise ValueError("Root node must be defined.")
        self.root = root
        self.vessels = []
        self.nodes = [root]
        self.collision_radius = 0.1


    def check_collision(self, x, y, parent_node):
        margin = 0.02  # Aumentando a margem de segurança
        new_position = np.array([x, y])

        for node in self.nodes:
            dist = np.linalg.norm(node.position() - new_position)
            if dist < (self.collision_radius + margin):
                print(f"Colisão detectada com nó na posição ({node.x}, {node.y})")
                return True

        for vessel in self.vessels:
            if self.do_edges_intersect(parent_node.position(), new_position, vessel.parent.position(), vessel.child.position()):
                print(f"Colisão detectada com o vaso entre ({vessel.parent.x}, {vessel.parent.y}) e ({vessel.child.x}, {vessel.child.y})")
                return True

        return False

    def add_terminal_node(self, x, y, flow=1.0):
        if len(self.nodes) == 1:  # Primeiro nó além da raiz
            new_node = Node(x, y, flow, self.root)
            new_vessel = Vessel(self.root, new_node)
            self.vessels.append(new_vessel)
            self.nodes.append(new_node)
            self.kd_tree = KDTree([new_vessel.get_medium_point()])
            return new_node

        # Encontre o vaso mais próximo para adicionar o novo nó ao ponto médio
        # nearest_vessel = self.find_nearest_vessel(x, y)
        # if nearest_vessel is None:
        #     print(f"Nenhum vaso válido encontrado para o nó na posição ({x}, {y}).")
        #     return None
        
        nearest_vessel = self.find_nearest_non_colliding_parent(x, y)
        if nearest_vessel is None:
            print(f"Nenhum vaso válido encontrado para o nó na posição ({x}, {y}).")
            return None

        # Conectar ao ponto médio do vaso
        mid_x, mid_y = nearest_vessel.get_medium_point()
        parent_node = Node(mid_x, mid_y, 0, nearest_vessel.parent)
        nearest_vessel.child.parent = parent_node
        nearest_vessel.usedMedium = True

        new_node = Node(x, y, flow, parent_node)
        new_vessel1 = Vessel(parent_node, nearest_vessel.child)
        new_vessel2 = Vessel(parent_node, new_node)

        self.vessels.append(new_vessel1)
        self.vessels.append(new_vessel2)
        self.nodes.append(parent_node)
        self.nodes.append(new_node)

        self.kd_tree = KDTree([vessel.get_medium_point() for vessel in self.vessels])

        self.local_optimization(new_node)

        print(f"Nó na posição ({x}, {y}) adicionado com sucesso.")
        return new_node

    def find_nearest_vessel(self, x, y):
        # Encontre o vaso mais próximo com base no ponto médio
        distances, indices = self.kd_tree.query((x, y), k=len(self.vessels))
        if isinstance(indices, np.int64):
            indices = [indices]
        for index in indices:
            vessel = self.vessels[index]
            if not vessel.usedMedium:
                vessel.usedMedium = True
                return vessel
        return None

    def print_tree_structure(self, node=None, level=0):
        if node is None:
            node = self.root
            print(f"Nó raiz na posição ({node.x}, {node.y}) com fluxo {node.flow}")
        else:
            print("  " * level + f"Nó filho de ({node.parent.x}, {node.parent.y}) na posição ({node.x}, {node.y}) com fluxo {node.flow}")
        
        for child in node.children:
            self.print_tree_structure(child, level + 1)

    def update_all_flows(self):
        def somar_fluxos_pos_ordem(node):
            if not node.children:  # Nó folha
                print(f"Nó folha na posição ({node.x}, {node.y}) com fluxo {node.flow}")
                return node.flow
            
            # Somar o fluxo dos filhos
            fluxo_total = 0
            for child in node.children:
                fluxo_total += somar_fluxos_pos_ordem(child)
            
            # Atualiza o fluxo do nó atual com o total acumulado dos filhos
            node.flow = fluxo_total
            print(f"Atualizando nó na posição ({node.x}, {node.y}): Fluxo total dos filhos = {node.flow}")
            return node.flow

        somar_fluxos_pos_ordem(self.root)


    def find_nearest_node(self, x, y):
        # Encontre o nó mais próximo (ponto médio)
        distances, indices = self.kd_tree.query((x, y), k=len(self.vessels))
        if isinstance(indices, np.int64):
            indices = [indices]
        for index in indices:
            vessel = self.vessels[index]
            if not vessel.usedMedium:
                vessel.usedMedium = True
                return vessel.parent
        return None

    def find_valid_parent_node(self, x, y):
        distances, indices = self.kd_tree.query((x, y), k=len(self.vessels))
        if isinstance(indices, np.int64):  # Verificar se indices é um único valor
            indices = [indices]  # Converter para lista
        for index in indices:
            vessel = self.vessels[index]
            if vessel.usedMedium:
                continue
            xn, yn = vessel.get_medium_point()
            potential_parent = Node(xn, yn, self.vessels[index].medium_point_flow)
            if not self.check_collision(x, y, potential_parent):
                vessel.usedMedium = True
                potential_parent.children.append(vessel.child)
                potential_parent.children.append(vessel.parent)
                new_vessel = Vessel(potential_parent, vessel.child)
                self.vessels[index].child = potential_parent
                self.vessels.append(new_vessel)
                self.nodes.append(potential_parent)
                return potential_parent
        return None
    
    def find_non_colliding_parents_in_vessel(self, x, y):
        non_colliding_parents = []
        for vessel in self.vessels:
            if not self.check_collision(x, y, vessel.medium_point_x, vessel.medium_point_y):
                non_colliding_parents.append(vessel)
        if len(non_colliding_parents) == 0:
            print ("DEU RUIM")
        return non_colliding_parents
    
    def check_collision(self, x0, y0, x1, y1):
        for vessel in self.vessels:
            v1x1, v1y1 = vessel.parent.position()
            v1x2, v1y2 = vessel.child.position()
            if self.do_edges_intersect([x0, y0], [x1, y1], vessel.parent.position(), vessel.child.position()):
                return True
        return False
    
    def verify_intersection(self, x0, y0, x1, y1, x2, y2, x3, y3):
        line1 = LineString([(x0, y0), (x1, y1)])
        line2 = LineString([(x2, y2), (x3, y3)])
        return line1.intersects(line2)
    
    def find_nearest_non_colliding_parent(self, x, y):
        non_colliding_parents = self.find_non_colliding_parents_in_vessel(x, y)
        if len(non_colliding_parents) == 0:
            return None
        
        nearest_value = 999999
        nearest_parent = None
        for parent in non_colliding_parents:
            if not parent.usedMedium:
                xn, yn = parent.get_medium_point()
                node1_aux = Node (x, y, 0)
                node2_aux = Node (xn, yn, 0)
                vessel_aux = Vessel(node1_aux, node2_aux)
                if vessel_aux.length < nearest_value:
                    nearest_parent = parent
                    nearest_value = vessel_aux.length
        return nearest_parent
    
            

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

    def plot_tree_with_flows(self):
        plt.figure(figsize=(12, 12))
        for vessel in self.vessels:
            x_values = [vessel.parent.x, vessel.child.x]
            y_values = [vessel.parent.y, vessel.child.y]
            plt.plot(x_values, y_values, color='blue', linewidth=vessel.diameter * 2)

        # Agora vamos imprimir os fluxos diretamente nos nós
        for node in self.nodes:
            plt.text(node.x, node.y, f'{node.flow:.1f}', color='green', fontsize=10, ha='center')

        plt.scatter([node.x for node in self.nodes], [node.y for node in self.nodes], color='red', s=10)
        plt.xlim(0, 1)
        plt.ylim(0, 1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()


    def print_all_nodes(self, node=None, level=0):
        if node is None:
            node = self.root
        
        # Print the current node with indentation based on its level in the tree
        print("  " * level + f"Node at ({node.x}, {node.y}) with flow {node.flow}")
        
        # Recursively print all children nodes
        for child in node.children:
            self.print_all_nodes(child, level + 1)

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
