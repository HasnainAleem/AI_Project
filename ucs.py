from collections import deque
import heapq
from collections import defaultdict                         #importing  libraraies
import networkx as nx
from queue import PriorityQueue


"""
This code is for uniform cost search, in which this fucntion calculates path from start to goal
and at the end it gives us the final cost took to reach the path, this code was my lab's code that you 
gave us built-in, i changed it to fit to GUI properly.
"""

class Graph_ucs:                                            #class for uniform cost search
    def __init__(self, directed=False):
        self.graph = defaultdict(list)
        self.heuristics = {}
        self.directed = directed

    def add_edge(self, node1, node2, weight):
        self.graph[node1].append((node2, weight))

    def set_heuristics(self, heuristics):
        self.heuristics = heuristics

    def uniform_cost_search(self, start, goals):
        visited = set()
        queue = [(0, start, [])]  # (cost, current_node, path)

        while queue:
            cost, current_node, path = heapq.heappop(queue)
            if current_node in visited:
                continue

            path = path + [current_node]

            if current_node in goals:
                return path, cost, current_node

            visited.add(current_node)

            neighbors = self.graph[current_node]
            for neighbor, weight in neighbors:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + weight, neighbor, path))

        return None, float('inf'), None

    @staticmethod
    def print_path(path, goal):
        if path:
            print(' -> '.join(path), end='')
            print()
            print(f' -> {goal}')
        else:
            print("No path found.")