from croblink import *
from pd_controller import PDController
from dfs_pathdinder import DFSPathfinder


# Turn | Move
STEERING = 0
MOVING = 1

# Compass Values for each direction 
# -180 to 180 --> If the robot is facing the right (EAST) the compass is 0
NORTH = 90
SOUTH = -90
WEST = 180
EAST = 0

# MAX / MIN Velocity values 
MAX_POW = 0.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

TIME_STEP = 0.005 # Sampling time 50 ms --> 0.005 

# Throttle PD Controller values
KP = 0.35 # Perfect
KD = 0 # No need

# Steering PD Controller Values
KPS = 0.02 # Perfect 
KDS = 0.00003 # 0.00005

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

        # Robot direction 
        self.previous_direction = None
        self.current_direction = None 
        self.direction_setpoint = None 

        # Cell to visit
        self.cell = None
    
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()


        self.readSensors()
        self.initial_position = (self.measures.x, self.measures.y)
        self.dfs.initialize(self.initial_position)

        moving_or_turning = STEERING

        while True:

            print("\n----------------------------------------\n")

            # Update previous measures
            self.previous_position = self.current_position
            self.previous_direction = self.current_direction
            
            self.readSensors()

            # Update current measures
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

            if moving_or_turning == STEERING and self.direction_setpoint is not None:
                print("Entrei no Turn")
                if self.steering():
                    print("Virei")
                    continue
                moving_or_turning = MOVING

            if moving_or_turning == MOVING and self.position_setpoint is not None:
                print("Entrei no Move")
                if self.move(): 
                    print("Movi")
                    continue
                moving_or_turning = STEERING
            
            print("Não virei e não movi")
            if self.direction_setpoint is not None and self.steering() : continue # Verify if moving did not change the direction 
            print("Não virei e não movi e não virei")
            print("Vou pedir Next Move")

            # REQUEST NEXT MOVE
            next_move = self.dfs.get_next_move(self.current_position, self.current_direction, ir_sensors)
            if next_move: 
                self.direction_setpoint, self.position_setpoint, self.cell = next_move  
                print(f"Next Move: {next_move}")
            else: 
                print("Map exploration is complete")
                break # Map exploration is complete


    def steering(self):
        if self.previous_direction == self.current_direction == self.direction_setpoint or \
            (self.previous_direction == self.current_direction and self.current_direction in [180, -180] and self.direction_setpoint in [180, -180]):
            self.driveMotors(0, 0) # Stop motors
            return False

        steering_correction = self.steering_pd_controller.compute_angle(self.current_direction, self.direction_setpoint)
        self.driveMotors(-steering_correction, steering_correction)
        print(f"Steering Power: ({-steering_correction}, {steering_correction})")
        return True
    

    def move(self):
        if self.previous_position == self.current_position == self.position_setpoint or \
            (
                self.previous_position == self.current_position and \
                self.robot_inside_cell()
            ):

            self.driveMotors(0, 0) # Stop Motors
            return False
            
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
        print(f"Throttle Power: ({motor_power}, {motor_power})")
        

    def robot_inside_cell(self):
        x, y = self.current_position
        (bl_x, bl_y), (tr_x, tr_y) = self.cell.coordinates # Bottom Left and Top Right

        return (bl_x <= x < tr_x) and (bl_y <= y < tr_y)