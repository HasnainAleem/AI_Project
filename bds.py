from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue


"""
This code is of the Bi-Directional search, it starts searching from both of the sides
from start and from the goal also. And at some point both searches meet at the middle
that point is called intersection point, when searches meet, the code stops and return the path.
"""

class Graph_bds:
    def __init__(self, directed=False):
        self.graph = {}
        self.directed = directed

    def add_edge(self, node1, node2, weight=1):
        if node1 not in self.graph:
            self.graph[node1] = {}
        if node2 not in self.graph:
            self.graph[node2] = {}

        self.graph[node1][node2] = weight
        if not self.directed:
            self.graph[node2][node1] = weight

    @staticmethod
    def print_path(path1, path2):
        path1.reverse()
        path1.extend(path2)
        print(' -> '.join(path1))

    def bidirectional_search(self, start, goal):
        queue1 = [(start, [start])]
        queue2 = [(goal, [goal])]

        explored1 = set()
        explored2 = set()

        while queue1 and queue2:
            node1, path1 = queue1.pop(0)
            node2, path2 = queue2.pop(0)

            if node1 == node2:
                self.print_path(path1, path2)
                return

            if node1 not in explored1:
                explored1.add(node1)
                for neighbor in self.graph[node1]:
                    new_path = list(path1)
                    new_path.append(neighbor)
                    queue1.append((neighbor, new_path))

            if node2 not in explored2:
                explored2.add(node2)
                for neighbor in self.graph[node2]:
                    new_path = list(path2)
                    new_path.append(neighbor)
                    queue2.append((neighbor, new_path))

        print("No path found between the start and goal nodes.")