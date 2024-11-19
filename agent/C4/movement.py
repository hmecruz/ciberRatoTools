import math

from constants import *
from pd_controller import PDController

class MovementModel:
    def __init__(self, robot_state, wheel_distance: float = 1):
        self.robot_state = robot_state
        self.wheel_distance = wheel_distance # Robot diameter (1 in the CiberRato environment)
        
        self.out_left = 0
        self.out_right = 0
        
        self.input_signal = 0

    @staticmethod
    def compute_out(input_signal, out_prev):
        return (max(min(input_signal, 0.15), -0.15) + out_prev) / 2
    

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
    

    def update_out(self):
        # Compute new outputs for the wheels
        self.out_left = self.compute_out(self.input_signal, self.out_left)
        self.out_right = self.compute_out(self.input_signal, self.out_right)
    

    def update_position(self):
        # Compute linear component
        linear_vel = self.compute_linear_velocity(self.out_left, self.out_right)

        # Compute position
        position = self.compute_position(self.robot_state.current_position, self.robot_state.current_direction, linear_vel)

        # TODO Apply filter

        # Update position 
        self.robot_state.current_position = position


    def update_direction(self, direction):        
        # Calculate direction based on formulas
        """
        # Compute rotational component
        rotational_vel = self.compute_rotational_velocity(out_left, out_right, self.wheel_distance)

        # Compute direction
        direction = self.compute_direction(self.robot_state.current_direction, rotational_vel)
        """

        # TODO Apply filter

        # Update direction
        self.robot_state.current_direction = direction if direction != -180 else 180 # Normalize direction

