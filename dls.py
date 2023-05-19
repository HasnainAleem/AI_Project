from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue

"""
This is the code for depth limited search, it is also the advanced form of depth first search, that searches
for the goal until the given limit is reached. If goal is not found in given depth, it returns none else 
it returns the path
"""

class Graph_dls:
    def __init__(self, directed=False):
        self.graph = {}
        self.directed = directed

    def add_edge(self, node1, node2, weight):
        if node1 not in self.graph:
            self.graph[node1] = {}
        if node2 not in self.graph:
            self.graph[node2] = {}
        self.graph[node1][node2] = weight
        if not self.directed:
            self.graph[node2][node1] = weight

    def depth_limited_search(self, current, goal, depth_limit = 4, path=[]):
        path.append(current)

        if current == goal:
            return path

        if depth_limit == 0:
            return None

        for neighbor in self.graph[current]:
            if neighbor not in path:
                new_path = self.depth_limited_search(neighbor, goal, depth_limit - 1, path)
                if new_path is not None:
                    return new_path

        path.pop()
        return None