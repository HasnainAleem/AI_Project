from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue

"""
This is the code for a star search, that uses hueristics of the vertices in order to navigate and reach the goal state.
This program/technique also considers hueristics of previous nodes in order to decide where to move.
"""

class Graph_astar:
    def __init__(self, directed=False):
        self.graph = {}
        self.heuristics = {}
        self.directed = directed

    def add_edge(self, node1, node2, weight):
        if node1 not in self.graph:
            self.graph[node1] = []
        self.graph[node1].append((node2, weight))

        if not self.directed:
            if node2 not in self.graph:
                self.graph[node2] = []
            self.graph[node2].append((node1, weight))

    def set_heuristics(self, heuristics):
        self.heuristics = heuristics

    def a_star_search(self, start, goals):
        open_list = [(0, start, [])]
        closed_list = set()

        while open_list:
            cost, node, path = min(open_list)
            open_list = [item for item in open_list if item[1] != node]  # Remove current node from open list
            path.append(node)

            if node in goals:
                return path, cost, node

            closed_list.add(node)

            neighbors = self.graph.get(node, [])
            for neighbor, weight in neighbors:
                if neighbor not in closed_list:
                    g = cost + weight
                    h = self.heuristics.get(neighbor)

                    if (g, neighbor) in open_list:
                        existing_item = next(item for item in open_list if item[1] == neighbor)
                        if g < existing_item[0]:
                            open_list.remove(existing_item)
                            open_list.append((g, neighbor, path[:]))
                    else:
                        open_list.append((g, neighbor, path[:]))

                return None, None, None

    @staticmethod
    def print_path(path, cost, goal):
        print(' -> '.join(path))
        print("Goal state is:", goal)
        print("Total cost:", cost)