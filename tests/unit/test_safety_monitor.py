from unittest.mock import MagicMock

import pytest

from roverstack.perception import ObstacleMap, PerceptionSystem, SensorFault
from roverstack.safety_monitor import SafetyMonitor, SafetyViolation


def test_check_command_allows_speed_within_limit():
    monitor = SafetyMonitor(max_speed=2.0)
    monitor.check_command(1.5)  # should not raise


@pytest.mark.parametrize("speed", [2.1, -2.1, 100.0])
def test_check_command_rejects_speed_over_limit(speed):
    monitor = SafetyMonitor(max_speed=2.0)
    with pytest.raises(SafetyViolation):
        monitor.check_command(speed)


def test_check_sensor_passes_through_a_healthy_reading():
    monitor = SafetyMonitor(max_speed=1.0)
    perception = PerceptionSystem()
    result = monitor.check_sensor(perception)
    assert isinstance(result, ObstacleMap)


def test_check_sensor_escalates_to_safety_violation_after_threshold():
    monitor = SafetyMonitor(max_speed=1.0, max_consecutive_sensor_faults=2)
    perception = PerceptionSystem()
    perception.inject_fault()
    with pytest.raises(SensorFault):
        monitor.check_sensor(perception)  # 1st fault in a row: not escalated yet
    with pytest.raises(SafetyViolation):
        monitor.check_sensor(perception)  # 2nd fault in a row: now it trips


def test_check_sensor_works_with_a_mocked_perception_system():
    mock_perception = MagicMock()
    mock_perception.scan.return_value = ObstacleMap(obstacles=frozenset())

    monitor = SafetyMonitor(max_speed=1.0)
    result = monitor.check_sensor(mock_perception)

    mock_perception.scan.assert_called_once()
    assert result.obstacles == frozenset()
