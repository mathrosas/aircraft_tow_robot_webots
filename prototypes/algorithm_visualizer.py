from heapq import heapify, heappush, heappop
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------------
# Create a Random Map for Demonstration
# -------------------------------------------
rows = 20
cols = 30
# 10% of cells are considered obstacles
map = np.random.rand(rows, cols) < 0.1

def getNeighbors_bfs(u):
    """
    Returns the 4-connected (up, down, left, right) neighbors
    that are free (no obstacle).
    """
    neighbors = []
    for delta in ((0,1), (1,0), (-1,0), (0,-1)):
        cand = (u[0] + delta[0], u[1] + delta[1])
        # Check bounds and if it's free
        if (0 <= cand[0] < len(map) and
            0 <= cand[1] < len(map[0]) and
            map[cand[0], cand[1]] < 0.3):
            neighbors.append(cand)
    return neighbors

def getNeighbors_dijk(u):
    """
    Returns the 8-connected neighbors with their "step" cost.
    If diagonal, step cost = sqrt(2), else = 1.
    Also checks if neighbor is free.
    """
    import math
    neighbors = []
    for delta in ((0,1), (0,-1), (1,0), (-1,0),
                  (1,1), (1,-1), (-1,1), (-1,-1)):
        cand = (u[0] + delta[0], u[1] + delta[1])
        if (0 <= cand[0] < len(map) and
            0 <= cand[1] < len(map[0]) and
            map[cand[0], cand[1]] < 0.3):
            dist = math.sqrt(delta[0]**2 + delta[1]**2)  # 1.0 or 1.4142...
            neighbors.append((dist, cand))
    return neighbors

# -------------------------------------------
# BFS (For Comparison)
# -------------------------------------------
def bfs(start, goal, map):
    """
    Standard BFS in a 4-connected grid.
    """
    queue = [start]
    visited = {start}
    parent = {}
    key = goal
    path = []

    plt.imshow(map) 
    plt.ion()
    plt.plot(goal[1], goal[0], 'y*')  # Mark goal with a yellow star

    while queue and (goal not in queue):
        v = queue.pop(0)
        for u in getNeighbors_bfs(v):
            if u not in visited:
                queue.append(u)
                visited.add(u)
                parent[u] = v
                # Visualization
                plt.plot(v[1], v[0], 'g*')
                plt.show()
                plt.pause(0.000001)

    # Reconstruct path if goal was reached
    while key in parent.keys():
        key = parent[key]
        path.insert(0, key)
    path.append(goal)

    # Plot the path
    for p in path:
        plt.ioff()
        plt.plot(p[1], p[0], 'r.')

    plt.show()

# -------------------------------------------
# Dijkstra (For Comparison)
# -------------------------------------------
def dijkstra(start, goal, map):
    """
    Dijkstra in an 8-connected grid. 
    Distances stored in a dictionary and updated via a min-heap.
    """
    queue = [(0, start)]
    heapify(queue)

    distances = defaultdict(lambda: float("inf"))
    distances[start] = 0

    visited = set()
    parent = {}
    key = goal
    path = []

    plt.imshow(map)
    plt.ion()
    plt.plot(goal[1], goal[0], 'y*')  # Mark goal

    while queue and (queue[0][1] != goal):
        (currentdist, v) = heappop(queue)
        if v in visited:
            continue
        visited.add(v)

        # Explore neighbors
        for (step_cost, u) in getNeighbors_dijk(v):
            if u not in visited:
                newcost = distances[v] + step_cost
                if newcost < distances[u]:
                    distances[u] = newcost
                    parent[u] = v
                    heappush(queue, (newcost, u))
                    plt.plot(v[1], v[0], 'g*')
                    plt.show()
                    plt.pause(0.000001)

    # Reconstruct path
    while key in parent.keys():
        key = parent[key]
        path.insert(0, key)
    path.append(goal)

    # Visualize path
    for p in path:
        plt.ioff()
        plt.plot(p[1], p[0], 'r.')

    plt.show()

# -------------------------------------------
# A* Implementation
# -------------------------------------------
def astar(start, goal, map):
    """
    A* search in an 8-connected grid.
    Uses Euclidean distance to the goal as a heuristic.
    """
    import math

    # Min-heap: store tuples of (f, node), where
    #   f = g(node) + h(node)
    #   g(node) is the distance from start
    #   h(node) is the heuristic to the goal
    queue = [(0, start)]
    heapify(queue)

    # Distances from start (g-values)
    g = defaultdict(lambda: float("inf"))
    g[start] = 0

    visited = set()
    parent = {}

    plt.imshow(map) 
    plt.ion()
    plt.plot(goal[1], goal[0], 'y*')  # Mark goal with a yellow star

    while queue:
        # current f-value and node
        (current_f, v) = heappop(queue)
        if v in visited:
            continue
        visited.add(v)

        # If we reached the goal, stop
        if v == goal:
            break

        # For each neighbor:
        for (step_cost, u) in getNeighbors_dijk(v):
            if u not in visited:
                tentative_g = g[v] + step_cost
                if tentative_g < g[u]:
                    g[u] = tentative_g
                    # Heuristic = Euclidean distance to goal
                    h = math.sqrt((goal[0] - u[0])**2 + (goal[1] - u[1])**2)
                    f = tentative_g + h
                    parent[u] = v
                    heappush(queue, (f, u))
                    plt.plot(v[1], v[0], 'g*')
                    plt.show()
                    plt.pause(0.000001)

    # Reconstruct path if we reached goal
    path = []
    key = goal
    while key in parent.keys():
        path.insert(0, key)
        key = parent[key]
    path.insert(0, start)  # Insert the start at the front if it isn't there
    # Or if we didn't find a path, 'path' will just be start->... or remain empty

    # Visualize final path
    for p in path:
        plt.ioff()
        plt.plot(p[1], p[0], 'r.')

    plt.show()

# -------------------------------------------
# Test any of the algorithms
# -------------------------------------------
start = (0, 0)
goal = (10, 10)
print("Start:", start)
print("Goal:", goal)

# bfs(start, goal, map)
# dijkstra(start, goal, map)
astar(start, goal, map)