import pytest

from roverstack.grid import Point
from roverstack.perception import PerceptionSystem, SensorFault


def test_scan_reports_static_obstacles():
    perception = PerceptionSystem(static_obstacles=frozenset({Point(1, 1)}))
    obstacle_map = perception.scan()
    assert obstacle_map.is_occupied(Point(1, 1))
    assert not obstacle_map.is_occupied(Point(0, 0))


def test_inject_fault_causes_scan_to_raise():
    perception = PerceptionSystem()
    perception.inject_fault()
    with pytest.raises(SensorFault):
        perception.scan()
