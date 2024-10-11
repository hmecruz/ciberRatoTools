from croblink import *
import time
from pid_controller import PIDController
from dfs import DFS

NORTH = -90
SOUTH = 90
WEST = [-180, 180]
EAST = 0

# Simulation Map
# Coordinates Map would be 14x28
CELLROWS=7
CELLCOLS=14

# MAX / MIN Velocity values 
MAX_POW = 0.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

TIME_STEP = 0.005 # Sampling time 50 ms --> 0.005 

# Throttle PID Controller values
KP = 0.002 # 0.002
KI = 0 
KD = 0

# Steering PID Controller Values
KPS = 0.095 
KIS = 0
KDS = 0.00075 # 0.0008

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
        # Depth First Search
        self.dfs = DFS() # returns next move

        # PIDController 
        self.speed_pid_controller = PIDController(kp=KP, ki=KI, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Throttle
        self.steering_pid_controller = PIDController(kp=KPS, ki=KIS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Steering
        
        self.speed_setpoint = None
        self.steering_setpoint = None
        
        # Robot Direction
        self.current_direction = EAST # Current robot direction
        self.next_direction = None # Direction the robot should move to
        #self.measures.compass=0.0 # -180 to 180 --> If the robot is facing the right (EAST) the compass is 0

        # Robot Position
        self.measures.x = 0.0  # 2 coordinates is the equivalent of 1 square in the simulation map 
        self.measures.y = 0.0  # 2 coordinates is the equivalent of 1 square in the simulation map 
        
    
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()
        
        while True:
            self.readSensors()

            self.next_direction = self.dfs()

            if self.first == True: 
                self.target_x = self.measures.x + 2
                self.first = False
                self.print_position_data()
            if self.measures.x >= self.target_x: 
                self.print_position_data()
                self.driveMotors(0, 0)
                self.target_x = self.measures.x + 2
                time.sleep(1)
            else:    
                self.driveMotors(MAX_POW, MAX_POW)

    def turn(self): 
        
        # Robot should turn to the correct way in the quickest way possible
        pass
        
            
    def print_position_data(self):
        """Print the robot's position, compass, and direction with a custom message."""
        print(f"X-Coordinate: {self.measures.x}")
        print(f"Y-Coordinate: {self.measures.y}")
        print(f"Compass: {self.measures.compass}")
        