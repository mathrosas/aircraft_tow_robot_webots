from scipy.spatial import distance
import math
import numpy as np
from grid import OccupancyGridMap
from a_star import astar_in_graph

# ------------------------------------------------------------------------------
# RRT Implementation for a 2D grid
# ------------------------------------------------------------------------------
def step_from_to(p1, p2, Dq):
    """
    Move from p1 towards p2 by step size Dq (straight line).
    """
    dist = distance.euclidean(p1, p2)
    if dist < Dq:
        return p2
    else:
        theta = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        return (p1[0] + Dq*np.cos(theta), p1[1] + Dq*np.sin(theta))

def rrt_search(grid_map: OccupancyGridMap, qstart: (int,int), qgoal: (int,int)) -> list:
    """
    Build a simple RRT in an obstacle-free manner (no collision checks) until we can 
    connect (or get near) the goal. Then we run a quick A* in the graph to get the path.
    
    :param grid_map: OccupancyGridMap
    :param qstart: start node (map coords)
    :param qgoal: goal node (map coords)
    :return: list of (x, y) from start to goal in map coords
    """
    import random

    # Graph adjacency: G[node] = [list of connected neighbors]
    G = {qstart: []}
    parent = {}
    # Step size in map coordinates
    Dq = 10  
    # Maximum sampling iterations to prevent infinite loops
    max_iters = 2000  

    x_max, y_max = grid_map.x_dim, grid_map.y_dim

    # If start == goal, trivially return
    if qstart == qgoal:
        return [qstart]

    # Build RRT until we can connect to (or near) the goal
    for _ in range(max_iters):
        # random sample in the grid
        qrand = (random.randint(0, x_max-1), 
                 random.randint(0, y_max-1))

        # Find the closest node in G to qrand
        closest_node = min(G, key=lambda x: distance.euclidean(x, qrand))
        # Move from closest_node to qrand by a step
        qnew = step_from_to(closest_node, qrand, Dq)

        # Round to integer map coords (since our grid is int-based)
        qnew = (int(round(qnew[0])), int(round(qnew[1])))

        # If it's not already in G, add
        if qnew not in G:
            G[qnew] = []
            G[closest_node].append(qnew)
            parent[qnew] = closest_node

        # Check if qnew is within Dq of qgoal
        if distance.euclidean(qnew, qgoal) < Dq:
            # Connect qgoal as well
            if qgoal not in G:
                G[qgoal] = []
            G[qnew].append(qgoal)
            parent[qgoal] = qnew
            break

    # If goal never got connected, we can just skip or do partial tree search
    if qgoal not in G:
        print("RRT: Could not connect to goal. Returning partial path or empty list.")
        return []

    # -------------
    # Final Path: 
    # We now run a tiny A* on the graph G from qstart->qgoal 
    # to find an actual path (shortest in terms of sum of Euclidean edges).
    # -------------
    path = astar_in_graph(qstart, qgoal, G)
    return path