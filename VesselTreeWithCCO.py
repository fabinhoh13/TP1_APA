import numpy as np
from Tree import Tree
from Node import Node
from Vessel import Vessel


def generate_terminal_points(num_points, space_size=1.0):
    points = np.random.rand(num_points, 2) * space_size
    flows = np.ones(num_points)  # Fixar o fluxo em 1 para todos os terminais
    return points, flows

def main():
    num_terminals = 50  # Ajustando para N terminais
    root = Node(0.1, 0.1, flow=num_terminals)
    tree = Tree(root)

    terminal_points, _ = generate_terminal_points(num_terminals)

    for point in terminal_points:
        flow = 1
        print(f"Trying to add node at ({point[0]}, {point[1]}) with flow {flow}.")
        tree.add_terminal_node(point[0], point[1], flow)

    tree.plot_tree()  # Gera a primeira imagem

    # Gerar a segunda imagem mostrando os fluxos
    tree.plot_tree_with_flows()


if __name__ == "__main__":
    main()
