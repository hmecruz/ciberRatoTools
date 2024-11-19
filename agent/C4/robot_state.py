from constants import *
from maze_map import Cell
from movement import MovementModel

class RobotState:
    def __init__(self):

        # Robot Position
        self.initial_position = None  # Y
        self.previous_position = None # Y
        self.current_position = None  # Y
        self.position_setpoint = None # N

        # Robot direction 
        self.previous_direction = None # Y
        self.current_direction = None  # Y
        self.direction_setpoint = None # N

        # Robot IR Sensors
        self.ir_sensors = None # Y

        # Robot mode
        self.steering_mode = True 
        self.moving_mode = False
       
        # Cell state
        self.cell = None          # Y
        self.cell_setpoint = None # Y
        self.cell_index = (0, 0)
        
        # Pathfinding
        self.pathfinding_path = []

        # Target Cells
        self.first_target_cell = None
        self.second_target_cell = None
        self.target_cell_path = []
        
       
    def initialize(self, robot): 
        robot.readSensors() # Read sensors
        self.initial_position = (robot.measures.x, robot.measures.y)
        
        bottom_left = (self.initial_position[0] - 1, self.initial_position[1] - 1)
        self.cell = Cell(bottom_left)
        self.cell_setpoint = self.cell
        
        robot.maze.add_cell_map(self.cell, self.cell_index)


    def read_sensors_update_measures(self, robot):
        # Update previous measures
        self.previous_position = self.current_position
        self.previous_direction = self.current_direction
        
        # Read upcoming data
        robot.readSensors()

        # Update current measures
        self.current_position = (robot.measures.x, robot.measures.y)
        self.current_direction = robot.measures.compass if robot.measures.compass != -180 else 180
        self.ir_sensors = {
            "center": robot.measures.irSensor[0],
            "left": robot.measures.irSensor[1],
            "right": robot.measures.irSensor[2],
            "back": robot.measures.irSensor[3]
        }

        # Print measures
        #print(f"Previous Position: {self.previous_position}")
        #print(f"Current Position: {self.current_position}")
        #print(f"Target Position: {self.position_setpoint}")
        #print(f"Previous Direction: {self.previous_direction}")
        #print(f"Current Direction: {self.current_direction}")
        #print(f"Target Direction: {self.direction_setpoint}")


    def switch_to_moving(self):
        self.steering_mode = False
        self.moving_mode = True
    

    def switch_to_steering(self):
        self.moving_mode = False
        self.steering_mode = True
    

    def sensor_vector_map(self, sensor_name):
        sensor_map = {
            NORTH: {
                "center": MOVE_NORTH,
                "left": MOVE_WEST,
                "right": MOVE_EAST,
                "back": MOVE_SOUTH,
            },
            WEST: {
                "center": MOVE_WEST,
                "left": MOVE_SOUTH,
                "right": MOVE_NORTH,
                "back": MOVE_EAST,
            },
            EAST: {
                "center": MOVE_EAST,
                "left": MOVE_NORTH,
                "right": MOVE_SOUTH,
                "back": MOVE_WEST,
            },
            SOUTH: {
                "center": MOVE_SOUTH,
                "left": MOVE_EAST,
                "right": MOVE_WEST,
                "back": MOVE_NORTH,
            },
        }
        return sensor_map.get(self.current_direction, {}).get(sensor_name)
    