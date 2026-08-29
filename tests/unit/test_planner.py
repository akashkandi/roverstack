import pytest

from roverstack.grid import GridBounds, Point
from roverstack.perception import ObstacleMap
from roverstack.planner import PathNotFoundError, a_star


def make_map(obstacles=frozenset()):
    return ObstacleMap(obstacles=frozenset(obstacles))


def test_a_star_finds_a_direct_path_with_no_obstacles():
    bounds = GridBounds(width=5, height=5)
    path = a_star(Point(0, 0), Point(2, 0), bounds, make_map())
    assert path[0] == Point(0, 0)
    assert path[-1] == Point(2, 0)
    assert len(path) == 3


def test_a_star_routes_around_a_wall():
    bounds = GridBounds(width=5, height=5)
    wall = {Point(1, y) for y in range(4)}
    obstacle_map = make_map(wall)
    path = a_star(Point(0, 0), Point(2, 0), bounds, obstacle_map)
    for point in path:
        assert not obstacle_map.is_occupied(point)


def test_a_star_raises_when_goal_is_boxed_in():
    bounds = GridBounds(width=3, height=3)
    goal = Point(1, 1)
    box = set(goal.neighbors())
    with pytest.raises(PathNotFoundError):
        a_star(Point(0, 0), goal, bounds, make_map(box))


@pytest.mark.parametrize(
    "start, goal",
    [
        (Point(-1, 0), Point(1, 1)),  # start out of bounds
        (Point(0, 0), Point(9, 9)),  # goal out of bounds
    ],
)
def test_a_star_rejects_out_of_bounds_points(start, goal):
    bounds = GridBounds(width=3, height=3)
    with pytest.raises(ValueError):
        a_star(start, goal, bounds, make_map())
