from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue

"""
This is the code for iterative deepening search, this is advanced form of depth first
in order to search goal by increasing it's limit after every iteration and at the end it 
returns us the path
"""

class Graph_ids:
    def __init__(self, directed=False):
        self.graph = {}
        self.directed = directed

    def add_edge(self, node1, node2, weight=1):
        if node1 in self.graph:
            self.graph[node1][node2] = weight
        else:
            self.graph[node1] = {node2: weight}

        if not self.directed:
            if node2 in self.graph:
                self.graph[node2][node1] = weight
            else:
                self.graph[node2] = {node1: weight}

    def depth_limited_search(self, current, goal, depth_limit):
        if current == goal:
            return [current]

        if depth_limit <= 0:
            return None

        for neighbor in self.graph.get(current, []):
            path = self.depth_limited_search(neighbor, goal, depth_limit - 1)
            if path is not None:
                return [current] + path

        return None

    def iterative_deepening_search(self, start, goals):
        depth_limit = 0

        while True:
            for goal in goals:
                path = self.depth_limited_search(start, goal, depth_limit)
                if path is not None:
                    return path, len(path) - 1, goal

            depth_limit += 1


    @staticmethod
    def print_path(path, goal):
        print(' -> '.join(path), end=' -> ')
        print("Goal is:")
        print(goal)