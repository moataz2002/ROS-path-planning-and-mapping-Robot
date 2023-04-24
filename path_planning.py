import numpy as np

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(grid, start, end):
    start_node = Node(start)
    end_node = Node(end)
    open_list = [start_node]
    closed_list = []

    while open_list:
        current_node = min(open_list, key=lambda node: node.f)
        open_list.remove(current_node)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = tuple(map(sum, zip(current_node.position, new_position)))

            if not (0 <= node_position[0] < grid.shape[0] and 0 <= node_position[1] < grid.shape[1]):
                continue

            if grid[node_position[0]][node_position[1]] == 1:
                continue

            children.append(Node(node_position, current_node))

        for child in children:
            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = sum(abs(a - b) for a, b in zip(child.position, end_node.position))
            child.f = child.g + child.h

            if child in open_list and child.g >= open_list[open_list.index(child)].g:
                continue

            open_list.append(child)

grid = np.array([
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
])

start = (0, 0)
end = (9, 9)

path = astar(grid, start, end)
print("Path:", path)
