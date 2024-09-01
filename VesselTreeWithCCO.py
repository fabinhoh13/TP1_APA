import numpy as np
from Tree import Tree
from Node import Node
from Vessel import Vessel


def generate_terminal_points(num_points, space_size=1.0):
    points = np.random.rand(num_points, 2) * space_size
    flows = np.random.rand(num_points) * 10
    return points, flows

def main():
    num_terminals = 100
    root = Node(0.1, 0.1, flow=num_terminals)
    tree = Tree(root)

    terminal_points, terminal_flows = generate_terminal_points(num_terminals)

    for point, flow in zip(terminal_points, terminal_flows):
        print(f"Trying to add node at ({point[0]}, {point[1]}) with flow {flow}.")
        tree.add_terminal_node(point[0], point[1], flow)

    tree.plot_tree()

if __name__ == "__main__":
    main()
