MOVE_NORTH = (0, 2)
MOVE_SOUTH = (0, -2)
MOVE_WEST = (-2, 0)
MOVE_EAST = (2, 0)

DIR_NORTH = 90
DIR_SOUTH = -90
DIR_WEST = [-180, 180]
DIR_EAST = 0

class DFSPathfinder:
    def __init__(self):
        self.visited = set()  # Set of visited positions
        self.stack = []  # Stack to keep track of DFS path (for backtracking)
        self.directions = [MOVE_NORTH, MOVE_EAST, MOVE_SOUTH, MOVE_WEST]  # Prioritized directions
        self.direction_mapping = {
            MOVE_NORTH: DIR_NORTH,
            MOVE_EAST: DIR_EAST,
            MOVE_SOUTH: DIR_SOUTH,
            MOVE_WEST: DIR_WEST
        }  # Map movement vectors to compass values

    def initialize(self, initial_position):
        """Initialize DFS with the robot's starting position."""
        self.stack.append(initial_position)
        self.visited.add(initial_position)
    
    def get_next_move(self, current_position, current_direction, ir_sensors):
        """
        Perform the DFS logic and return the next move for the robot.
        
        Arguments:
        - current_position: The current (x, y) position of the robot.
        - current_direction: The current compass direction the robot is facing (NORTH, SOUTH, EAST, WEST).
        - ir_sensors: Dictionary containing the IR sensor values.

        Returns:
        - A tuple containing the direction (as a movement vector) and the next position to move to, or None if no move is possible.
        """
        
        # Check for unvisited neighbors in the prioritized direction order
        for direction in self.directions:
            next_position = (current_position[0] + direction[0], current_position[1] + direction[1])
            
            # Check if this neighbor has already been visited
            if next_position not in self.visited:
                # Check if the move is valid using the IR sensors
                if self.is_valid_move(current_direction, direction, ir_sensors):
                    self.visited.add(next_position)
                    self.stack.append(current_position)  # Push the current position to the stack before moving
                    return self.direction_mapping[direction], next_position  # Return the direction and the next position to move to
        return None
    
    def is_valid_move(self, current_direction, target_direction, ir_sensors):
        
        # Define threshold for obstacle detection (adjust as needed)
        obstacle_threshold = 1.5
        
        # Determine which sensor to check based on current and target directions
        if current_direction == DIR_NORTH:
            if target_direction == MOVE_NORTH:
                return ir_sensors["center"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["back"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["left"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["right"] < obstacle_threshold

        elif current_direction == DIR_SOUTH:
            if target_direction == MOVE_NORTH:
                return ir_sensors["back"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["center"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["right"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["left"] < obstacle_threshold

        elif current_direction == DIR_EAST:
            if target_direction == MOVE_NORTH:
                return ir_sensors["left"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["right"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["back"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["center"] < obstacle_threshold

        elif current_direction == DIR_WEST:
            if target_direction == MOVE_NORTH:
                return ir_sensors["right"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["left"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["center"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["back"] < obstacle_threshold
        
        # If the direction is not matched, return False as default
        return False
        