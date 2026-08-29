"""Simulated perception (sensor) system.

A real robot uses a camera/lidar to build a map of what's around it. We
simulate that here with a simple set of "occupied" grid cells, but keep the
same shape of interface (scan() returns a map, and it can be made to fail)
so the testing technique transfers directly to a real sensor pipeline.
"""

from dataclasses import dataclass, field

from roverstack.grid import Point


class SensorFault(Exception):
    """Raised when the simulated sensor pipeline fails to produce a reading."""


@dataclass
class ObstacleMap:
    obstacles: frozenset

    def is_occupied(self, point: Point) -> bool:
        return point in self.obstacles


@dataclass
class PerceptionSystem:
    static_obstacles: frozenset = field(default_factory=frozenset)
    _fault_injected: bool = field(default=False, init=False, repr=False)

    def inject_fault(self) -> None:
        """Simulate the sensor failing from this point forward (e.g. a lidar dropout)."""
        self._fault_injected = True

    def clear_fault(self) -> None:
        self._fault_injected = False

    def scan(self, dynamic_obstacles: frozenset = frozenset()) -> ObstacleMap:
        if self._fault_injected:
            raise SensorFault("perception pipeline lost signal (simulated dropout)")
        return ObstacleMap(obstacles=self.static_obstacles | dynamic_obstacles)
