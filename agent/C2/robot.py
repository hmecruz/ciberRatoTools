from croblink import *
from low_pass_filter import LowPassFilter
from pd_controller import PDController
from dfs_pathfinding import DFSPathfinder


STEERING = 0
MOVE = 1

# Position values to move for each direction
# 2 coordinates is the equivalent of 1 square in the simulation map 
MOVE_NORTH = (0, 2)
MOVE_SOUTH = (0, -2)
MOVE_WEST = (-2, 0)
MOVE_EAST = (2, 0)

# Compass Values for each direction 
# -180 to 180 --> If the robot is facing the right (EAST) the compass is 0
NORTH = 90.0
SOUTH = -90.0
WEST = [-180.0, 180.0]
EAST = 0.0

# Simulation Map
# Coordinates Map would be 14x28
CELLROWS=7
CELLCOLS=14

# MAX / MIN Velocity values 
MAX_POW = 0.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

TIME_STEP = 0.005 # Sampling time 50 ms --> 0.005 

# Throttle PD Controller values
KP = 0.35 
KD = 0 

# Steering PD Controller Values
KPS = 0.02 
KDS = 0.00003

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
        # PIDController 
        self.speed_pid_controller = PDController(kp=KP, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Throttle
        self.direction_pid_controller = PDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Steering
        
        # Robot direction and position
        self.initial_position = None # Robot initial position
        self.current_position = None # Robot current position
        self.current_direction = None # Robot current direction

        self.previous_position = None
        self.previous_direction = None
        
        self.position_setpoint = None # # Target position
        self.direction_setpoint = None # Target direction

        # Error Threshold
        self.position_error_threshold = 0.2 
        self.direction_error_threshold = 2

        #  Movemnt Flag
        self.movement_flag = STEERING

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
            self.previous_position = self.current_position
            self.previous_direction = self.current_direction

            self.readSensors()
            self.current_position = (self.measures.x, self.measures.y)
            self.current_direction = self.measures.compass

            print(f"Previous Position: {self.previous_position}")
            print(f"Current Position: {self.current_position}")
            print(f"Target Position: {self.position_setpoint}")
            print(f"Previous Direction: {self.previous_direction}")
            print(f"Current Direction: {self.current_direction}")
            print(f"Target Direction: {self.direction_setpoint}")

            ir_sensors = {
                "center": self.measures.irSensor[0],
                "left": self.measures.irSensor[1],
                "right": self.measures.irSensor[2],
                "back": self.measures.irSensor[3]
            } 

            if self.movement_flag == STEERING:
                if self.turn():
                    print("\n----------------------------------------\n")
                    continue
            if self.movement_flag == MOVE:
                if self.move():
                    print("\n----------------------------------------\n")
                    continue
                 
            # Request DFS for the next move
            next_move = self.dfs.get_next_move(self.current_position, self.current_direction, ir_sensors)
            if next_move: 
                self.direction_setpoint, self.position_setpoint = next_move  
            else: 
                break # Map exploration is complete

            #print(f"Next Move: {next_move}")

            print("\n----------------------------------------\n")
            

    def turn(self):
        if self.direction_setpoint is None or \
            self.previous_direction == self.current_direction == self.direction_setpoint: 
            
            self.driveMotors(0, 0) # Stop Motors
            self.movement_flag = MOVE # STEERING IS COMPLETED
            return False # Movement is completed
        
        steering_correction = self.direction_pid_controller.compute(self.current_direction, self.direction_setpoint)
        self.driveMotors(-steering_correction, steering_correction) 
        print(f"Steering Power: ({-steering_correction}, {steering_correction})")
        
        return True
    
    def move(self):
        if self.position_setpoint is None or \
            self.previous_position == self.current_position == self.position_setpoint or (
            self.previous_position == self.current_position and \
                self.is_within_threshold(self.current_position[0], self.position_setpoint[0], self.position_error_threshold) and \
                self.is_within_threshold(self.current_position[1], self.position_setpoint[1], self.position_error_threshold)):
            
            self.driveMotors(0, 0) # Stop Motors
            self.movement_flag = STEERING # MOVE IS COMPLETED
            return False # Movement is completed
            
        if not self.is_within_threshold(self.current_direction, self.direction_setpoint, self.direction_error_threshold):
            self.movement_flag = STEERING # MOVE IS NOT COMPLETED BUT REQUIRES A TURNING ADJUST
            return True # Movement is not completed
         
        current_direction = self.get_direction() 
        invert_power = current_direction in [SOUTH, WEST]  # Invert motor power if facing SOUTH or WEST
        if current_direction in (WEST, EAST):
            self.move_to_position(self.current_position[0], self.position_setpoint[0], invert_power) # x coordinate
        elif current_direction in (NORTH, SOUTH):
            self.move_to_position(self.current_position[1], self.position_setpoint[1], invert_power) # y coordinate

        return True # Movement is not completed

    def move_to_position(self, current_position, position_setpoint, invert_power): 
        motor_power = self.speed_pid_controller.compute(current_position, position_setpoint)

        if invert_power:
            motor_power = -motor_power  # Reverse motor power if robot is facing SOUTH or WEST

        self.driveMotors(motor_power, motor_power)    
        print(f"Throttle Power: ({motor_power}, {motor_power})")


    def get_direction(self):
        """Get the robot's direction within a threshold."""
        directions = {
            'NORTH': NORTH,
            'SOUTH': SOUTH,
            'WEST': WEST,
            'EAST': EAST
        }

        for direction_name, direction_value in directions.items():
            if isinstance(direction_value, list):
                # Check for WEST direction which has two values
                if direction_value[0] + self.direction_error_threshold <= self.current_direction <= direction_value[0] or \
                direction_value[1] - self.direction_error_threshold <= self.current_direction <= direction_value[1]:
                    return direction_value
            else:
                if abs(self.current_direction - direction_value) < self.direction_error_threshold:
                    return direction_value

        return None  # If no direction matched
    
    def is_within_threshold(self, current_value, setpoint_value, threshold):
        """Check if the current value is within the specified threshold of the setpoint."""
        return abs(current_value - setpoint_value) <= threshold
    
    
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
     