import math

from constants import *
from utils.noise_filter import *
from utils.KalmanFilter import *


def reflect_angle(angle):
    angle = angle % 360
    
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    
    if angle > 90:
        return 180 - angle
    elif angle < -90:
        return -180 - angle
    
    return angle

def normalize_angle(angle):
    if angle < 0:
        return - (180+angle)
    elif angle > 0:
        return 180 - angle
    else:
        return 180


class MovementModel:
    def __init__(self, robot_state, wheel_distance: float = 1):
        self.robot_state = robot_state
        self.wheel_distance = (
            wheel_distance  # Robot diameter (1 in the CiberRato environment)
        )

        self.out_left = 0
        self.out_right = 0

        self.input_signal_left = 0
        self.input_signal_right = 0

        # TODO: remove this if u want, just for testing and plots
        self.compass_movement_model = 0

        self.angle_kalman_filter = AngleKalmanFilter()

    @staticmethod
    def compute_out(input_signal, out_prev):
        return (max(min(input_signal, 0.15), -0.15) + out_prev) / 2

    @staticmethod
    def compute_linear_velocity(out_left, out_right):
        return (out_left + out_right) / 2

    @staticmethod
    def compute_position(prev_pos, prev_direction_degrees, linear_vel):
        x_prev, y_prev = prev_pos
        prev_direction_radians = math.radians(
            prev_direction_degrees
        )  # Convert degrees to radians
        x_new = x_prev + linear_vel * math.cos(prev_direction_radians)
        y_new = y_prev + linear_vel * math.sin(prev_direction_radians)
        return (x_new, y_new)

    @staticmethod
    def compute_rotational_velocity(out_left, out_right, wheel_distance):
        return (out_right - out_left) / wheel_distance

    @staticmethod
    def compute_direction(prev_direction_degrees, rotational_vel):
        prev_direction_radians = math.radians(
            prev_direction_degrees
        )  # Convert degrees to radians

        direction_radians = prev_direction_radians + rotational_vel

        direction = math.degrees(direction_radians)

        return direction

    def update_out(self):
        # Compute new outputs for the wheels
        self.out_left = self.compute_out(self.input_signal_left, self.out_left)
        self.out_right = self.compute_out(self.input_signal_right, self.out_right)

    def update_position(self):
        # Compute linear component
        linear_vel = self.compute_linear_velocity(self.out_left, self.out_right)

        # Compute position
        position = self.compute_position(
            self.robot_state.current_position,
            self.robot_state.current_direction,
            linear_vel,
        )

        self.robot_state.current_position = position

    def update_direction(self, compass):
        # Calculate direction based on formulas

        # Compute rotational component
        rotational_vel = self.compute_rotational_velocity(
            self.out_left, self.out_right, self.wheel_distance
        )

        # Compute direction
        self.compass_movement_model = self.compute_direction(
            self.robot_state.current_direction, rotational_vel
        )

        must_reflect =  (self.compass_movement_model < -90 or self.compass_movement_model > 90) and (compass < -90 or compass > 90)

        if must_reflect:
            self.compass_movement_model = reflect_angle(self.compass_movement_model)
            compass = reflect_angle(compass)



        if not self.angle_kalman_filter.firstTime:
            if must_reflect:
                self.compass_movement_model = reflect_angle(self.compass_movement_model)

            self.angle_kalman_filter.predict(
                np.array([[self.compass_movement_model], [self.out_left], [self.out_right]]),
                np.array([[self.input_signal_left], [self.input_signal_right]]),
            )
        else:
            self.angle_kalman_filter.firstTime = False

        self.angle_kalman_filter.update(np.array([[compass]]))
        
        filtered_compass, predicted_compass = self.angle_kalman_filter.get_estimate()
        

        filtered_compass = int(np.round(filtered_compass[0]))


        if must_reflect:
            filtered_compass = normalize_angle(filtered_compass)
            self.compass_movement_model = normalize_angle(self.compass_movement_model)
        

        # Update direction
        self.robot_state.current_direction = (
            filtered_compass if filtered_compass != -180 else 180
        )  # Normalize direction
