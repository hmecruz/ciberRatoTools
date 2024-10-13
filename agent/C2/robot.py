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

# Simulation Map
# Coordinates Map would be 14x28
CELLROWS=7
CELLCOLS=14

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
        self.direction_pid_controller = PDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Steering
        
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

        # DFS Pathfinder
        self.dfs = DFSPathfinder()

    
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.readSensors()
        self.initial_position = (self.measures.x, self.measures.y)
        self.dfs.initialize(self.initial_position)

        while True:

            self.readSensors()
            self.current_position = (self.measures.x, self.measures.y)
            self.current_direction = self.measures.compass

            print(f"Current Position: {self.current_position}")
            print(f"Target Position: {self.position_setpoint}")
            print(f"Current Direction: {self.current_direction}")
            print(f"Target Direction: {self.direction_setpoint}")

            ir_sensors = {
                "center": self.measures.irSensor[0],
                "left": self.measures.irSensor[1],
                "right": self.measures.irSensor[2],
                "back": self.measures.irSensor[3]
            } 

            if self.move(): 
                print("\n----------------------------------------\n")
                continue # Skip iteration until robot finishes moving

            # Request DFS for the next move
            next_move = self.dfs.get_next_move(self.current_position, self.current_direction, ir_sensors)
            if next_move: 
                self.direction_setpoint, self.position_setpoint = next_move  
            else: 
                break # Map exploration is complete

            #print(f"Next Move: {next_move}")

            print("\n----------------------------------------\n")
            


    def move(self):
        if self.direction_setpoint is not None and self.current_direction != self.direction_setpoint:
            self.turn()  

        elif self.position_setpoint is not None and self.current_position != self.position_setpoint:
            if self.current_direction in (DIR_EAST, DIR_WEST):
                self.move_to_position(self.current_position[0], self.position_setpoint[0]) # x coordinate
            elif self.current_direction in (DIR_NORTH, DIR_SOUTH):
                self.move_to_position(self.current_position[1], self.position_setpoint[1]) # y coordinate

        else:
            self.driveMotors(0, 0)  # Stop motors
            return False  # Movement is complete

        return True  # Still turning or moving                


    def turn(self): 
        steering_correction = self.direction_pid_controller.compute(self.current_direction, self.direction_setpoint)
        self.driveMotors(-steering_correction, steering_correction) 
        print(f"Steering Power: ({-steering_correction}, {steering_correction})")
        
    def move_to_position(self, current_position, position_setpoint): # Special type current position and position setpoint is a single value, x or y
        
        motor_power = self.speed_pid_controller.compute(current_position, position_setpoint)

        if self.current_direction == DIR_SOUTH or self.current_direction == DIR_WEST:
            motor_power = -motor_power  # Reverse motor power for those directions

        self.driveMotors(motor_power, motor_power)    
        print(f"Throttle Power: ({motor_power}, {motor_power})")
    
    def print_position_data(self):
        """Print the robot's position, compass, and direction with a custom message."""
        print(f"X-Coordinate: {self.measures.x}")
        print(f"Y-Coordinate: {self.measures.y}")
        print(f"Compass: {self.measures.compass}")

    def print_obstacle_sensors(self, center_sensor, left_sensor, right_sensor):
        """Prints the values from the obstacle sensors."""
        print(f"Center IR Sensor: {center_sensor}") 
        print(f"Left IR Sensor: {left_sensor}")  
        print(f"Right IR Sensor: {right_sensor}")  
     