"""Ties perception, planning, control, and the safety monitor into a single
step-able mission simulation.
"""

from dataclasses import dataclass

from roverstack.controller import PIDController
from roverstack.grid import GridBounds
from roverstack.perception import PerceptionSystem
from roverstack.planner import PathNotFoundError, a_star
from roverstack.safety_monitor import SafetyMonitor, SafetyViolation


@dataclass
class MissionResult:
    success: bool
    path_taken: list
    stopped_reason: str
    steps_taken: int


@dataclass
class Simulator:
    bounds: GridBounds
    perception: PerceptionSystem
    controller: PIDController
    safety_monitor: SafetyMonitor
    max_steps: int = 200

    def run_mission(self, start, goal, dynamic_obstacles=frozenset()):
        self.controller.reset()
        position = start
        path_taken = [position]

        for step in range(1, self.max_steps + 1):
            try:
                obstacle_map = self.safety_monitor.check_sensor(self.perception, dynamic_obstacles)
            except SafetyViolation as exc:
                return MissionResult(False, path_taken, str(exc), step - 1)

            try:
                planned_path = a_star(position, goal, self.bounds, obstacle_map)
            except PathNotFoundError as exc:
                return MissionResult(False, path_taken, str(exc), step - 1)

            if position == goal:
                return MissionResult(True, path_taken, None, step - 1)

            next_waypoint = planned_path[1]
            error = float(position.manhattan_distance(goal))
            speed = self.controller.step(error)

            try:
                self.safety_monitor.check_command(speed)
            except SafetyViolation as exc:
                return MissionResult(False, path_taken, str(exc), step - 1)

            position = next_waypoint
            path_taken.append(position)

        return MissionResult(
            position == goal,
            path_taken,
            None if position == goal else "max_steps exceeded",
            self.max_steps,
        )
