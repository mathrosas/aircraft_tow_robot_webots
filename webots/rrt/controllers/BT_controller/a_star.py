import heapq
import math
from collections import defaultdict
from scipy.spatial import distance

def astar_in_graph(start, goal, G):
    """
    A* on an already-built adjacency dict G,
    where G[u] is a list of neighbors of u.
    We treat each edge cost as Euclidean distance. 
    """

    queue = [(0.0, start)]
    heapq.heapify(queue)
    dist = defaultdict(lambda: float('inf'))
    dist[start] = 0.0
    parent = {}
    visited = set()

    while queue:
        (current_f, v) = heapq.heappop(queue)
        if v in visited:
            continue
        visited.add(v)

        if v == goal:
            break

        for u in G[v]:
            step_cost = distance.euclidean(v, u)
            tentative_cost = dist[v] + step_cost
            if tentative_cost < dist[u]:
                dist[u] = tentative_cost
                f = tentative_cost + eucl_heuristic(u, goal)
                parent[u] = v
                heapq.heappush(queue, (f, u))

    # Reconstruct path if possible
    if dist[goal] == float('inf'):
        return []

    path = [goal]
    s = goal
    while s in parent:
        s = parent[s]
        path.append(s)
    path.reverse()
    return path

def eucl_heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)