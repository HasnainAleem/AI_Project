from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue


class Graph_ucs:
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
            print(f' -> {goal}')
        else:
            print("No path found.")


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
        print(goal)

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
                self.print_path(path1, path2, node1)
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




if __name__ == '_main_':
    # Create the graph
    graph_bfs = Graph_bfs(directed=False)
    graph_dfs = Graph_dfs(directed=False)
    graph_astar = Graph_astar(directed=False)
