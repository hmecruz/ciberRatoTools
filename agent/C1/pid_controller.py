class PIDController:
    def __init__(self, kp, ki, kd, time_step, max_output=None):
        self.kp = kp  # Proportional gain --> Corrections based on the current error
        self.ki = ki  # Integral gain --> Account for cumulative error over time. Applys a correction to reach the setpoint
        self.kd = kd  # Derivative gain --> Responds to the rate of change of the error. Reduce overshooting, helps dampen the systen and provide a smoother control. Can reduce the response time
        self.time_step = time_step
        self.error = 0
        self.previous_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.max_output = max_output
    
    def compute(self, current_value, setpoint):
        """Compute the control signal."""
        self.error = setpoint - current_value 
        self.integral_error += self.error * self.time_step # Sum of all the errors
        self.derivative_error = (self.error - self.previous_error) / self.time_step
        self.previous_error = self.error
        output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error
        if self.max_output != None:
            output = min(self.max_output, output)

        return output