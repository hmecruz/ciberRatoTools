from croblink import *
from pd_controller import PDController
from dfs_pathdinder import DFSPathfinder

# Constants for moving and turning
STEERING = 0
MOVING = 1

# Center Sensor Threshold for detecting impossible moves
CENTER_SENSOR_THRESHOLD = 3.5 

# Compass Values for each direction
NORTH = 90
SOUTH = -90
WEST = 180
EAST = 0

# Velocity limits
MAX_POW = 0.15
MIN_POW = -0.15

# Sampling Time
TIME_STEP = 0.005

# Throttle PD Controller values
KP = 0.35 
KD = 0 

# Steering PD Controller Values
KPS = 0.02 
KDS = 0.00003 

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
        # PDController 
        self.speed_pd_controller = PDController(kp=KP, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Throttle
        self.steering_pd_controller = PDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Steering

        # DFSPathfinder
        self.dfs = DFSPathfinder()

       # Robot Position
        self.initial_position = None
        self.previous_position = None
        self.current_position = None 
        self.position_setpoint = None 

        self.last_safe_position = None

        # Robot direction 
        self.previous_direction = None
        self.current_direction = None 
        self.direction_setpoint = None 

        # Cell
        self.cell = None # Current cell 
        self.cell_setpoint = None # Cell to visit
        
    
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.readSensors()
        self.initial_position = (self.measures.x, self.measures.y)
        self.dfs.initialize(self.initial_position)

        state = STEERING

        while True:

            #print("\n----------------------------------------\n")

            self.read_sensors_update_measures()
            ir_sensors = self.get_ir_sensors_readings()

            if state == STEERING and self.direction_setpoint is not None:
                #print("Entrei no Turn")
                if self.steering():
                    #print("Virei")
                    continue
                state = MOVING

            if state == MOVING and self.position_setpoint is not None:
                #print("Entrei no Move")
                if self.move(): 
                    #print("Movi")
                    continue
                state = STEERING
            
            #print("Não virei e não movi")
            if self.direction_setpoint is not None and self.steering(): # Verify if moving did not change the direction 
                continue # Verify if moving did not change the direction 
            
            self.last_safe_position = self.current_position # Save position if not able to perform next move
            self.cell = self.cell_setpoint

            #print("Não virei e não movi e não virei")
            #print("Vou pedir Next Move")

            # REQUEST NEXT MOVE
            next_move = self.dfs.get_next_move(self.current_position, self.current_direction, ir_sensors)
            if next_move: 
                self.direction_setpoint, self.position_setpoint, self.cell_setpoint = next_move  
                print(f"Next Move: {self.direction_setpoint}, {self.position_setpoint}, {self.cell_setpoint.coordinates}")
                print("\n----------------------------------------\n")
            else: 
                print("Map exploration is complete")
                break # Map exploration is complete


    def steering(self):
        if self.previous_direction == self.current_direction == self.direction_setpoint:
            self.driveMotors(0, 0) # Stop motors
            return False

        steering_correction = self.steering_pd_controller.compute_angle(self.current_direction, self.direction_setpoint)
        self.driveMotors(-steering_correction, steering_correction)
        #print(f"Steering Power: ({-steering_correction}, {steering_correction})")
        return True
    

    def move(self):
        print(f"Center Sensor: {self.get_ir_sensors_readings().get("center", 0.0)}")
        if self.previous_position == self.current_position == self.position_setpoint or \
            (
                self.previous_position == self.current_position and \
                self.robot_inside_cell() and \
                self.get_ir_sensors_readings().get("center", 0.0) <= 2.8
                
            ):

            self.driveMotors(0, 0) # Stop Motors
            return False
        
        if self.is_robot_crashing():
            self.position_setpoint = self.last_safe_position
            self.cell_setpoint = self.cell
            
        invert_power = self.direction_setpoint in [SOUTH, WEST]
        if self.direction_setpoint in (WEST, EAST):
            self.move_to_position(self.current_position[0], self.position_setpoint[0], invert_power) # x coordinate
        elif self.direction_setpoint in (NORTH, SOUTH):
            self.move_to_position(self.current_position[1], self.position_setpoint[1], invert_power) # y coordinate
        
        return True

    def move_to_position(self, current_val, target_val, invert_power):
        motor_power = self.speed_pd_controller.compute(current_val, target_val)

        if invert_power:
            motor_power = -motor_power  # Reverse motor power if robot is facing SOUTH or WEST

        self.driveMotors(motor_power, motor_power)    
        #print(f"Throttle Power: ({motor_power}, {motor_power})")
        

    def robot_inside_cell(self):
        x, y = self.current_position
        (bl_x, bl_y), (tr_x, tr_y) = self.cell_setpoint.coordinates
        return (bl_x <= x < tr_x) and (bl_y <= y < tr_y)
    

    def read_sensors_update_measures(self):
        # Update previous measures
        self.previous_position = self.current_position
        self.previous_direction = self.current_direction
        
        # Read upcoming data
        self.readSensors()

        # Update current measures
        self.current_position = (self.measures.x, self.measures.y)
        self.current_direction = self.measures.compass if self.measures.compass != -180 else 180
        
        # Print measures
        #print(f"Previous Position: {self.previous_position}")
        #print(f"Current Position: {self.current_position}")
        #print(f"Target Position: {self.position_setpoint}")
        #print(f"Previous Direction: {self.previous_direction}")
        #print(f"Current Direction: {self.current_direction}")
        #print(f"Target Direction: {self.direction_setpoint}")

    def is_robot_crashing(self):
        ir_sensors = self.get_ir_sensors_readings()
        if ir_sensors['center'] > CENTER_SENSOR_THRESHOLD:
            print(ir_sensors['center'])
            return True
        return False

    def get_ir_sensors_readings(self):
        return {
            "center": self.measures.irSensor[0],
            "left": self.measures.irSensor[1],
            "right": self.measures.irSensor[2],
            "back": self.measures.irSensor[3]
        }