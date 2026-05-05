import heapq
import math
import numpy as np
from grid import OccupancyGridMap

def dijkstra_search(grid_map: OccupancyGridMap, start: (int, int), goal: (int, int)) -> list:
    """
    Standard Dijkstra search on a 2D occupancy grid.
    :param grid_map: OccupancyGridMap instance (with map_data, x_dim, y_dim).
    :param start: (x,y) start in map coordinates
    :param goal: (x,y) goal in map coordinates
    :return: list of (x,y) forming path from start to goal (or empty if none)
    """

    # Distances and parents
    dist = np.full((grid_map.x_dim, grid_map.y_dim), np.inf, dtype=float)
    dist[start[0], start[1]] = 0.0

    parent = dict()  # to reconstruct the path
    visited = set()

    # Priority queue of (cost, (x,y))
    queue = [(0.0, start)]
    heapq.heapify(queue)

    # While there are still nodes to process
    while queue:
        current_dist, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            break  # We reached the goal

        # Explore neighbors
        for nxt in grid_map.succ(current, avoid_obstacles=True):
            # cost of moving from current -> nxt
            step_cost = cost_euclidean(current, nxt)
            new_cost = current_dist + step_cost
            if new_cost < dist[nxt[0], nxt[1]]:
                dist[nxt[0], nxt[1]] = new_cost
                parent[nxt] = current
                heapq.heappush(queue, (new_cost, nxt))

    # Reconstruct path if goal was reached and not infinite
    if dist[goal[0], goal[1]] == np.inf:
        # no known path
        return []

    # Reconstruct backwards
    path = [goal]
    s = goal
    while s in parent:
        s = parent[s]
        path.append(s)
    path.reverse()
    return path


def cost_euclidean(a: (int, int), b: (int, int)) -> float:
    """
    Simple Euclidean distance for a grid step, 
    especially if you allow 8-direction movements.
    """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)