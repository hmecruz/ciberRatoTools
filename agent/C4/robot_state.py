from constants import *
from utils.maze_map import Cell
from utils.movement import MovementModel
from utils.noise_filter import *


class RobotState:
    def __init__(self):

        # Robot Position
        self.initial_position = None
        self.previous_position = None
        self.current_position = (0, 0)
        self.position_setpoint = None

        # Robot direction
        self.previous_direction = None
        self.current_direction = 0
        self.direction_setpoint = None

        # Robot IR Sensors
        self.ir_sensors = None  # Y

        # Robot mode
        self.steering_mode = True
        self.moving_mode = False
        self.recalibration_mode = False

        # Recalibration
        self.recalibration_complete = False
        self.recalibration_phase = 0
        self.recalibration_counter_x = 0  # Counter till robot is able to recalibrate
        self.recalibration_counter_y = 0  # Counter till robot is able to recalibrate

        # Cell state
        self.cell = None
        self.cell_setpoint = None
        self.cell_index = (0, 0)

        # Pathfinding
        self.pathfinding_path = []

        # Target Cells
        self.target_cells = {} # id: CELL
        self.target_cell_path = None

        # Movement Model
        self.movement_model = MovementModel(self)

    def initialize(self, robot):
        robot.readSensors()  # Read sensors
        self.initial_position = (0, 0)

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

        if not robot.measures.start:
            return False

        # Update out, position and direction
        self.movement_model.update_out()
        self.movement_model.update_position()
        direction = robot.measures.compass if robot.measures.compass != -180 else 180
        self.movement_model.update_direction(direction)

        # Update IR obstacle sensors
        self.ir_sensors = {
            "center": robot.measures.irSensor[0],
            "left": robot.measures.irSensor[1],
            "right": robot.measures.irSensor[2],
            "back": robot.measures.irSensor[3],
        }

        return True

    def switch_to_moving(self):
        self.steering_mode = False
        self.moving_mode = True
        self.recalibration_mode = False

    def switch_to_steering(self):
        self.moving_mode = False
        self.steering_mode = True
        self.recalibration_mode = False

    def switch_to_recalibration(self):
        self.moving_mode = False
        self.steering_mode = False
        self.recalibration_mode = True

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
        return sensor_map.get(closest_direction(self.current_direction), {}).get(
            sensor_name
        )
