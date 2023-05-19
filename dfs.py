from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue

"""
This code is for the depth first search, this is also a searching technique, it searches from start to goal in one
direction, that is why it sometimes does not reach the goal and goes in infinte. But it reach the goal state also and 
at the end it returns us the path
"""

class Graph_dfs:
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

    def depth_first_search(self, start, goals):
        visited = set()
        stack = [(start, [])]

        while stack:
            node, path = stack.pop()
            if node not in visited:
                visited.add(node)
                path.append(node)

                if node in goals:
                    return path, node

                neighbors = self.graph.get(node, [])
                for neighbor in neighbors:
                    stack.append((neighbor, path[:]))

        return None, None

    @staticmethod
    def print_path(path, goal):
        print(' -> '.join(path))
        print("Goal state is:", goal)