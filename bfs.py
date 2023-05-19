from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue

"""
This code is for breadth first search, it is a simple technique to traverse a grpah in order.
Same functionality is performed by this code, it starts from one point and reaches goal traversing the nodes of the graph
"""

class Graph_bfs:
    def __init__(self, directed=False):
        self.graph = {}
        self.directed = directed

    def add_edge(self, node1, node2):
        if node1 not in self.graph:
            self.graph[node1] = []
        self.graph[node1].append(node2)

        if not self.directed:
            if node2 not in self.graph:
                self.graph[node2] = []
            self.graph[node2].append(node1)

    def breadth_first_search(self, start, goals):
        visited = set()
        queue = deque([(start, [])])

        while queue:
            node, path = queue.popleft()
            if node not in visited:
                visited.add(node)
                path.append(node)

                if node in goals:
                    return path, node

                neighbors = self.graph.get(node, [])
                for neighbor in neighbors:
                    queue.append((neighbor, path[:]))

        return None, None

    @staticmethod
    def print_path(path, goal):
        print(' -> '.join(path))
        print("Goal state is:", goal)