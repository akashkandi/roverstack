"""Independent safety monitor — deliberately kept separate and simple, so a
bug in perception/planning/control can't also take down the thing meant to
catch it.
"""

from dataclasses import dataclass

from roverstack.perception import SensorFault


class SafetyViolation(Exception):
    """Raised when a safety limit is breached; the mission must stop."""


@dataclass
class SafetyMonitor:
    max_speed: float
    max_consecutive_sensor_faults: int = 1

    _consecutive_faults: int = 0

    def check_sensor(self, perception, dynamic_obstacles=frozenset()):
        try:
            obstacle_map = perception.scan(dynamic_obstacles)
        except SensorFault:
            self._consecutive_faults += 1
            if self._consecutive_faults >= self.max_consecutive_sensor_faults:
                raise SafetyViolation(
                    f"sensor faulted {self._consecutive_faults} time(s) in a row "
                    f"(limit {self.max_consecutive_sensor_faults}); commanding stop"
                )
            raise
        else:
            self._consecutive_faults = 0
            return obstacle_map

    def check_command(self, speed: float) -> None:
        if abs(speed) > self.max_speed:
            raise SafetyViolation(
                f"commanded speed {speed} exceeds max_speed {self.max_speed}"
            )
