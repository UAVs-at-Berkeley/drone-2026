from typing import List, Tuple
import math
from itertools import permutations

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def total_distance(waypoints: List[Tuple], order):
    total = 0.0
    for i in range(len(order) - 1):
        lat1, lon1 = waypoints[order[i]][0], waypoints[order[i]][1]
        lat2, lon2 = waypoints[order[i + 1]][0], waypoints[order[i + 1]][1]
        total += haversine(lat1, lon1, lat2, lon2)
    
    return total

def tsp_waypoint_optimizer(waypoints: List[Tuple]):
    indices = list(range(len(waypoints)))

    shortest_dist = total_distance(waypoints, indices)
    shortest_order = indices
    for perm in permutations(indices):
        dist = total_distance(waypoints, perm)
        if dist < shortest_dist:
            shortest_dist, shortest_order = dist, perm
    
    return [waypoints[shortest_order[i]] for i in range(len(waypoints))]
