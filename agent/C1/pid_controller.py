class PIDController:
    def __init__(self, kp, kd, time_step, min_output=None, max_output=None):
        self.kp = kp  # Proportional gain --> Corrections based on the current error
        self.kd = kd  # Derivative gain --> Responds to the rate of change of the error. Reduce overshooting, helps dampen the systen and provide a smoother control. Can reduce the response time
        self.time_step = time_step
        self.error = 0
        self.previous_error = 0
        self.derivative_error = 0
        self.min_output = min_output
        self.max_output = max_output
    
    def compute(self, current_value, setpoint):
        """Compute the control signal."""
        self.error = setpoint - current_value 
        self.derivative_error = (self.error - self.previous_error) / self.time_step
        self.previous_error = self.error
        output = self.kp*self.error + self.kd*self.derivative_error
        if self.max_output is not None and self.min_output is not None:
            output = max(-0.05 + self.min_output, min(0.05 + self.max_output, output))

        return round(output, 2) # Round to two decimal cases