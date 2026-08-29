"""A* path planning over an occupancy grid."""

import heapq

from roverstack.grid import GridBounds, Point
from roverstack.perception import ObstacleMap


class PathNotFoundError(Exception):
    """Raised when no collision-free path exists between start and goal."""


def a_star(start: Point, goal: Point, bounds: GridBounds, obstacle_map: ObstacleMap):
    if not bounds.contains(start):
        raise ValueError(f"start {start} is out of bounds {bounds}")
    if not bounds.contains(goal):
        raise ValueError(f"goal {goal} is out of bounds {bounds}")
    if obstacle_map.is_occupied(start):
        raise ValueError(f"start {start} is occupied")
    if obstacle_map.is_occupied(goal):
        raise ValueError(f"goal {goal} is occupied")

    if start == goal:
        return [start]

    open_heap: list[tuple[int, int, Point]] = []
    counter = 0
    heapq.heappush(open_heap, (start.manhattan_distance(goal), counter, start))

    came_from: dict[Point, Point] = {}
    g_score = {start: 0}
    visited = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return _reconstruct_path(came_from, current)

        for neighbor in current.neighbors():
            if not bounds.contains(neighbor):
                continue
            if obstacle_map.is_occupied(neighbor):
                continue
            if neighbor in visited:
                continue

            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                counter += 1
                f_score = tentative_g + neighbor.manhattan_distance(goal)
                heapq.heappush(open_heap, (f_score, counter, neighbor))

    raise PathNotFoundError(f"no path from {start} to {goal}")


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
