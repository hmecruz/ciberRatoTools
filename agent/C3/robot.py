from croblink import *
from low_pass_filter import LowPassFilter
from pd_controller import PDController
from dfs_pathfinding import DFSPathfinder

# Position values to move for each direction
# 2 coordinates is the equivalent of 1 square in the simulation map 
MOVE_NORTH = (0, 2)
MOVE_SOUTH = (0, -2)
MOVE_WEST = (-2, 0)
MOVE_EAST = (2, 0)

# Compass Values for each direction 
# -180 to 180 --> If the robot is facing the right (EAST) the compass is 0
DIR_NORTH = 90
DIR_SOUTH = -90
DIR_WEST = [-180, 180]
DIR_EAST = 0

# MAX / MIN Velocity values 
MAX_POW = 0.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

TIME_STEP = 0.005 # Sampling time 50 ms --> 0.005 

# Throttle PD Controller values
KP = 0.3 # Perfect
KD = 0 # No need

# Steering PD Controller Values
KPS = 0.02 # Perfect 
KDS = 0 # No need

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
        # PIDController 
        self.speed_pid_controller = PDController(kp=KP, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Throttle
        self.steering_pid_controller = PDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Steering
        
        # Low Pass Filters for position and direction
        self.x_position_filter = LowPassFilter(window_size=5)  # Low-pass filter for x-coordinate
        self.y_position_filter = LowPassFilter(window_size=5)  # Low-pass filter for y-coordinate
        self.direction_filter = LowPassFilter(window_size=5)

        # Robot direction and position
        self.initial_position = None # Robot initial position
        self.current_position = None # Robot current position
        self.current_direction = None # Robot current direction
        
        self.position_setpoint = None # # Target position
        self.direction_setpoint = None # Target direction
    
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.readSensors()
        self.initial_position = (self.measures.x, self.measures.y)
        self.current_position = self.initial_position
        self.position_setpoint = tuple(map(sum, zip(self.initial_position, MOVE_WEST)))
        
        self.current_direction = self.measures.compass
        self.direction_setpoint = DIR_NORTH

        while True:
            
            # Throttle Test
            self.readSensors()
            self.current_position = (self.measures.x, self.measures.y)
            print(f"Current Position: {self.current_position}")
            print(f"Target Position: {self.position_setpoint}")
            speed_control_signal = self.speed_pid_controller.compute(self.current_position[0], self.position_setpoint[0])
            print(f"Control Signal: {speed_control_signal}")
            motor_power = speed_control_signal
            self.driveMotors(motor_power, motor_power)
            print(f"lPow rPow: ({motor_power}, {motor_power})")

            print("\n----------------------------------------\n")

        

            # Throttle Test
            self.readSensors()
            self.current_direction = self.measures.compass
            print(f"Current Direction: {self.current_direction}")
            print(f"Target Direction: {self.direction_setpoint}")
            steering_control_signal = self.steering_pid_controller.compute(self.current_direction, self.direction_setpoint)
            print(f"Control Signal: {steering_control_signal}")
            motor_power = steering_control_signal
            self.driveMotors(-motor_power, motor_power)
            print(f"lPow rPow: ({-motor_power}, {motor_power})")

            print("\n----------------------------------------\n")