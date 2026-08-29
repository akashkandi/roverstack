"""A minimal PID controller used to "drive" the rover along a planned path."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    output_limit: float = 1.0

    _integral: float = 0.0
    _prev_error: float | None = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def step(self, error: float, dt: float = 1.0) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")

        self._integral += error * dt
        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))
