import heapq
import math
from grid import OccupancyGridMap

# ------------------------------------------------------------------------------
# A* Implementation (for a 2D occupancy grid)
# ------------------------------------------------------------------------------
def a_star_search(grid_map: OccupancyGridMap, start: (int, int), goal: (int, int)) -> list:
    """
    A* path planning in a 2D occupancy grid.
    Uses Euclidean distance as a heuristic. 
    Return a list of (x, y) from start to goal (may be empty if no path is found).
    """

    # Store the cost from the start to each cell
    g = {}
    g[start] = 0.0

    # Parent dictionary for path reconstruction
    parent = {}

    # Priority queue of (f, (x, y))
    # f = g(node) + heuristic(node), 
    # where heuristic(node) = Euclidean distance to goal
    queue = [(0.0, start)]
    heapq.heapify(queue)

    visited = set()

    while queue:
        current_f, current = heapq.heappop(queue)

        if current in visited:
            continue
        visited.add(current)

        # If we've reached the goal, break
        if current == goal:
            break

        # Expand neighbors (avoiding obstacles)
        neighbors = grid_map.succ(current, avoid_obstacles=True)
        for nxt in neighbors:
            step_cost = cost_euclidean(current, nxt)  # typically 1 or sqrt(2)
            tentative_g = g[current] + step_cost

            if nxt not in g or tentative_g < g[nxt]:
                g[nxt] = tentative_g
                # Heuristic: Euclidean distance
                h = math.sqrt((goal[0] - nxt[0])**2 + (goal[1] - nxt[1])**2)
                f = tentative_g + h
                parent[nxt] = current
                heapq.heappush(queue, (f, nxt))

    # Reconstruct path if goal is reached
    if goal not in parent and goal != start:
        # no path found
        return []

    # If the start == goal or we found a path, reconstruct
    path = [goal]
    s = goal
    while s in parent:
        s = parent[s]
        path.append(s)
    path.reverse()
    return path


def cost_euclidean(a: (int, int), b: (int, int)) -> float:
    """
    Simple Euclidean distance for a grid step.
    If you use 8-connected expansions, this could be 1 or sqrt(2).
    """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)