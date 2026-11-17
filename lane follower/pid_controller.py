"""
PID Controller for Line Tracking Steering.
Computes smooth steering adjustments based on position error.
"""

import time
import config


class PIDController:
    """
    Proportional-Integral-Derivative (PID) Controller with anti-windup clamping.
    """

    def __init__(self,
                 kp=config.PID_KP,
                 ki=config.PID_KI,
                 kd=config.PID_KD,
                 max_integral=config.PID_MAX_INTEGRAL,
                 output_limits=(-50.0, 50.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.min_out, self.max_out = output_limits

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def reset(self):
        """Resets the controller state (useful when line is re-acquired)."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def update(self, error: float, dt: float = None) -> float:
        """
        Calculates PID output.
        :param error: Offset from center (-1.0 to +1.0)
        :param dt: Time delta in seconds. If None, computed automatically.
        :return: Steering correction output.
        """
        now = time.time()
        if dt is None:
            if self.last_time is None:
                dt = 0.033  # ~30 FPS initial guess
            else:
                dt = now - self.last_time
        self.last_time = now

        # Prevent division by zero or anomalous spikes
        if dt <= 0.0 or dt > 0.5:
            dt = 0.033

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        self.prev_error = error

        # Combined PID output
        output = p_term + i_term + d_term

        # Clamp output to limits
        output = max(self.min_out, min(self.max_out, output))
        return output
