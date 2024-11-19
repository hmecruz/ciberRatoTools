import math

from robot_state import RobotState

class MovementModel:
    def __init__(self, robot_state: RobotState, wheel_distance: float = 1):
        self.robot_state = robot_state
        self.wheel_distance = wheel_distance # Robot diameter (1 in the CiberRato environment)


    @staticmethod
    def compute_out(input_signal, out_t_prev):
        return (max(min(input_signal, 0.15), -0.15) + out_t_prev) / 2
    

    @staticmethod
    def compute_linear_velocity(out_left, out_right):
        return (out_left + out_right) / 2
    

    @staticmethod
    def compute_position(prev_pos, prev_direction_degrees, linear_vel):    
        x_prev, y_prev = prev_pos
        prev_direction_radians = math.radians(prev_direction_degrees) # Convert degrees to radians 
        x_new = x_prev + linear_vel * math.cos(prev_direction_radians)
        y_new = y_prev + linear_vel * math.sin(prev_direction_radians)
        return (x_new, y_new)
    

    @staticmethod
    def compute_rotational_velocity(out_left, out_right, wheel_distance):
        return (out_right - out_left) / wheel_distance


    @staticmethod
    def compute_direction(prev_direction_degrees, rotational_vel):
        prev_direction_radians = math.radians(prev_direction_degrees) # Convert degrees to radians 
        direction_radians = prev_direction_radians + rotational_vel
        return math.degrees(direction_radians) 
    

    def update_robot_state(self, input_signal, out_left_prev, out_right_prev):
        # Compute new outputs for the wheels
        out_left = self.compute_out(input_signal, out_left_prev)
        out_right = self.compute_out(input_signal, out_right_prev)

        # Update linear and rotational components
        linear_vel = self.compute_linear_velocity(out_left, out_right)
        rotational_vel = self.compute_rotational_velocity(out_left, out_right, self.wheel_distance)

        # Compute position and direction
        position = self.compute_position(self.robot_state.previous_position, self.robot_state.previous_direction, linear_vel)
        direction = self.compute_direction(self.robot_state.previous_direction, rotational_vel)
        
        # Update position and direction
        self.robot_state.current_position = position
        self.robot_state.current_direction = direction