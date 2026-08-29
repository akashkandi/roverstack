"""End-to-end scenario tests — running the ENTIRE stack through realistic
full missions, and checking the overall outcome, the way a safety
requirements document would read.
"""

from roverstack.controller import PIDController
from roverstack.grid import GridBounds, Point
from roverstack.perception import PerceptionSystem
from roverstack.safety_monitor import SafetyMonitor
from roverstack.simulator import Simulator


def build_simulator(
    static_obstacles=frozenset(),
    max_speed=10.0,
    max_consecutive_sensor_faults=1,
    width=10,
    height=10,
    max_steps=100,
):
    return Simulator(
        bounds=GridBounds(width=width, height=height),
        perception=PerceptionSystem(static_obstacles=frozenset(static_obstacles)),
        controller=PIDController(kp=1.0, ki=0.05, kd=0.1, output_limit=max_speed),
        safety_monitor=SafetyMonitor(
            max_speed=max_speed,
            max_consecutive_sensor_faults=max_consecutive_sensor_faults,
        ),
        max_steps=max_steps,
    )


def test_scenario_open_field_reaches_goal():
    sim = build_simulator()
    result = sim.run_mission(Point(0, 0), Point(9, 9))
    assert result.success
    assert result.path_taken[0] == Point(0, 0)
    assert result.path_taken[-1] == Point(9, 9)


def test_scenario_routes_around_a_wall_of_obstacles():
    wall = {Point(5, y) for y in range(8)}
    sim = build_simulator(static_obstacles=wall)
    result = sim.run_mission(Point(0, 0), Point(9, 9))
    assert result.success
    for point in result.path_taken:
        assert point not in wall


def test_scenario_sensor_dropout_causes_a_safe_stop_not_a_crash():
    sim = build_simulator(max_consecutive_sensor_faults=1)
    sim.perception.inject_fault()
    result = sim.run_mission(Point(0, 0), Point(9, 9))
    assert not result.success
    assert "sensor" in result.stopped_reason.lower()
    assert result.path_taken == [Point(0, 0)]  # never moved blind


def test_scenario_boxed_in_goal_fails_cleanly():
    goal = Point(5, 5)
    box = set(goal.neighbors())
    sim = build_simulator(static_obstacles=box)
    result = sim.run_mission(Point(0, 0), goal)
    assert not result.success
    assert "no path" in result.stopped_reason.lower()


def test_scenario_overly_aggressive_controller_is_stopped_by_safety_monitor():
    sim = Simulator(
        bounds=GridBounds(width=10, height=10),
        perception=PerceptionSystem(),
        controller=PIDController(kp=1000.0, ki=0.0, kd=0.0, output_limit=1000.0),
        safety_monitor=SafetyMonitor(max_speed=1.0),
        max_steps=50,
    )
    result = sim.run_mission(Point(0, 0), Point(9, 9))
    assert not result.success
    assert "exceeds max_speed" in result.stopped_reason
