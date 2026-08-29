import pytest

from roverstack.controller import PIDController


def test_proportional_only_scales_with_error():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, output_limit=100.0)
    assert pid.step(3.0) == pytest.approx(6.0)


def test_output_is_clamped_to_limit():
    pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limit=1.0)
    assert pid.step(50.0) == pytest.approx(1.0)
    assert pid.step(-50.0) == pytest.approx(-1.0)


def test_integral_term_accumulates_over_steps():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
    first = pid.step(1.0, dt=1.0)
    second = pid.step(1.0, dt=1.0)
    assert second > first  # constant positive error keeps growing the integral term


def test_reset_clears_internal_state():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
    pid.step(10.0)
    pid.reset()
    fresh = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
    assert pid.step(1.0) == pytest.approx(fresh.step(1.0))


def test_step_rejects_non_positive_dt():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    with pytest.raises(ValueError):
        pid.step(1.0, dt=0.0)
