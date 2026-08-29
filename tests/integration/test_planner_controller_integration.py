from roverstack.controller import PIDController
from roverstack.grid import GridBounds, Point
from roverstack.perception import ObstacleMap
from roverstack.planner import a_star


def test_following_the_planned_path_reduces_distance_to_goal_every_step():
    bounds = GridBounds(width=10, height=10)
    start, goal = Point(0, 0), Point(6, 6)
    path = a_star(start, goal, bounds, ObstacleMap(obstacles=frozenset()))

    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, output_limit=100.0)
    distances = [p.manhattan_distance(goal) for p in path]

    for i in range(len(distances) - 1):
        error = float(distances[i])
        speed = pid.step(error)
        assert speed >= 0
        assert distances[i + 1] < distances[i]
