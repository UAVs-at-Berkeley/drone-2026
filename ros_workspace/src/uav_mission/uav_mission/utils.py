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

def point_in_polygon(lat: float, lon: float, polygon: List[Tuple[float, float]]) -> bool:
    """Return True if (lat, lon) is inside a 2D polygon (ray-casting, even-odd rule)."""
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        lat_i, lon_i = polygon[i]
        lat_j, lon_j = polygon[j]
        if (lon_i > lon) != (lon_j > lon):
            lat_cross = (lat_j - lat_i) * (lon - lon_i) / (lon_j - lon_i) + lat_i
            if lat < lat_cross:
                inside = not inside
        j = i
    return inside


def tsp_waypoint_optimizer(waypoints: List[Tuple]):
    indices = list(range(len(waypoints)))

    shortest_dist = total_distance(waypoints, indices)
    shortest_order = indices
    for perm in permutations(indices):
        dist = total_distance(waypoints, perm)
        if dist < shortest_dist:
            shortest_dist, shortest_order = dist, perm
    
    return [waypoints[shortest_order[i]] for i in range(len(waypoints))]
