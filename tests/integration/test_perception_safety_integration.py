"""Integration tests: perception feeding the safety monitor.

Checks the real (non-mocked) PerceptionSystem + SafetyMonitor pair, including
the fault-injection path — this is the software equivalent of pulling a
sensor cable during a bench test.
"""

import pytest

from roverstack.perception import PerceptionSystem, SensorFault
from roverstack.safety_monitor import SafetyMonitor, SafetyViolation


def test_healthy_perception_flows_through_the_safety_monitor():
    perception = PerceptionSystem()
    monitor = SafetyMonitor(max_speed=1.0)
    obstacle_map = monitor.check_sensor(perception)
    assert obstacle_map is not None


def test_transient_fault_then_recovery_never_trips_safety():
    perception = PerceptionSystem()
    monitor = SafetyMonitor(max_speed=1.0, max_consecutive_sensor_faults=2)

    perception.inject_fault()
    with pytest.raises(SensorFault):
        monitor.check_sensor(perception)  # one fault, not escalated yet

    perception.clear_fault()
    monitor.check_sensor(perception)  # recovers cleanly, resets the streak


def test_persistent_fault_eventually_trips_a_safety_violation():
    perception = PerceptionSystem()
    monitor = SafetyMonitor(max_speed=1.0, max_consecutive_sensor_faults=3)
    perception.inject_fault()

    tripped = False
    for _ in range(5):
        try:
            monitor.check_sensor(perception)
        except SafetyViolation:
            tripped = True
            break
        except SensorFault:
            continue

    assert tripped
