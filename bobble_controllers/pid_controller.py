"""Simple PID controller with exponential output filter and anti-windup."""


class PIDController:
    """Simple PID with exponential output filter and anti-windup."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, output_limit=None,
                 max_integral=None, output_filter=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.max_integral = max_integral
        self.output_filter = output_filter  # 0=no filter, 1=frozen

        self._integral = 0.0
        self._last_output = 0.0

    def reset(self):
        self._integral = 0.0
        self._last_output = 0.0

    def compute(self, error, dt, derivative=None):
        """Compute PID output.

        Args:
            error: setpoint - actual
            dt: time step
            derivative: if provided, used directly for D term (e.g. gyro rate)
                        instead of differentiating error
        """
        # P term
        p_out = self.kp * error

        # I term
        self._integral += error * dt
        if self.max_integral is not None and self.max_integral > 0:
            self._integral = max(-self.max_integral,
                                 min(self.max_integral, self._integral))
        i_out = self.ki * self._integral

        # D term
        if derivative is not None:
            d_out = self.kd * derivative
        else:
            d_out = 0.0

        output = p_out + i_out + d_out

        # Exponential output filter: higher coeff = more smoothing
        if self.output_filter > 0.0:
            output = (self._last_output * self.output_filter +
                      output * (1.0 - self.output_filter))

        # Output limiting
        if self.output_limit is not None and self.output_limit > 0:
            output = max(-self.output_limit, min(self.output_limit, output))

        self._last_output = output
        return output
