from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue

"""
This is the code of best first search, it uses hueristics of vertices instead of weights in order to navigate 
to the goal and the same functionality is performed by this code
"""

class Graph_best_first_search:
    def __init__(self, directed=False):
        self.graph = nx.Graph() if not directed else nx.DiGraph()
        self.heuristics = {}

    def add_edge(self, node1, node2, weight=1):
        self.graph.add_edge(node1, node2, weight=weight)

    def set_heuristics(self, heuristics):
        self.heuristics = heuristics

    @staticmethod
    def print_path(path, goal):
        print(' -> '.join(path) + ' -> ' + goal)

    def best_first_search(self, start, goals):
        frontier = PriorityQueue()
        explored = set()

        # Add the start node to the frontier with a priority based on the heuristic value
        frontier.put((self.heuristics[start], start))
        goal = None

        while not frontier.empty():
            _, current_node = frontier.get()

            if current_node in goals:
                goal = current_node
                break

            explored.add(current_node)

            for neighbor in self.graph.neighbors(current_node):
                if neighbor not in explored and not frontier.contains(neighbor):
                    frontier.put((self.heuristics[neighbor], neighbor))

        if goal:
            traced_path = self.trace_path(start, goal, explored)
            return traced_path, goal

        return None, None

    def trace_path(self, start, goal, explored):
        path = []
        current_node = goal

        while current_node != start:
            path.append(current_node)

            neighbors = list(self.graph.neighbors(current_node))
            neighbors_heuristics = [self.heuristics[node] for node in neighbors if node in explored]
            min_heuristic = min(neighbors_heuristics)

            for neighbor in neighbors:
                if self.heuristics[neighbor] == min_heuristic:
                    current_node = neighbor
                    break

        path.append(start)
        path.reverse()
        return path