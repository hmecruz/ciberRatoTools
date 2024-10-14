MOVE_NORTH = (0, 2)
MOVE_SOUTH = (0, -2)
MOVE_WEST = (-2, 0)
MOVE_EAST = (2, 0)

NORTH = 90.0
SOUTH = -90.0
WEST = (-180.0, 180.0)
EAST = 0.0

#CELL = ([LEFT BOTTOM CORNER], [RIGHT TOP CORNER])
#CELL = ([x, y], [x+2, y+2])

class DFSPathfinder:
    def __init__(self):
        self.visited = set()  # Set of visited cells
        self.stack = []  # Stack to keep track of DFS path
        self.directions = [MOVE_NORTH, MOVE_EAST, MOVE_SOUTH, MOVE_WEST]  # Prioritized directions
        
        self.direction_mapping = {
            MOVE_NORTH: NORTH,
            MOVE_EAST: EAST,
            MOVE_SOUTH: SOUTH,
            MOVE_WEST: WEST[1]
        }  # Map movement vectors to compass values

        self.opposite_direction_mapping = {
            NORTH: SOUTH,
            EAST: WEST[1],
            SOUTH: NORTH,
            WEST: EAST
        }

    def initialize(self, initial_position):
        """Initialize DFS with the robot's starting position."""
        bottom_left = (initial_position[0] - 1, initial_position[1] - 1)
        top_right = (initial_position[0] + 1, initial_position[1] + 1)
        cell = (bottom_left, top_right)
        self.stack.append(cell)
        self.visited.add(cell)

    def create_cell_from_direction(self, direction):
        """Create a cell based on the robot's upcoming direction."""
        
        last_cell = self.stack[-1]  # Get the last visited cell
        (bl_x, bl_y), (tr_x, tr_y) = last_cell  # Unpack the last cell coordinates

         # Create a new cell based on the direction of movement
        if direction == NORTH: 
            bottom_left = (bl_x, tr_y + 0.1)
            top_right = (tr_x, tr_y + 2)
        elif direction == SOUTH: 
            bottom_left = (bl_x, bl_y - 2)
            top_right = (tr_x, bl_y -0.1)
        elif direction == WEST: 
            bottom_left = (bl_x - 2, bl_y)
            top_right = (bl_x - 0.1, tr_y)
        elif direction == EAST: 
            bottom_left = (tr_x + 0.1, bl_y)
            top_right = (tr_x + 2, tr_y)
            
        # Return the cell as a tuple of bottom-left and top-right corners
        return (bottom_left, top_right)
    
    def get_cell_middle_position(self, cell):
        """Calculate the middle position of a given cell."""
        (bl_x, bl_y), (tr_x, tr_y) = cell
        return (tr_x - 1, tr_y - 1)
       
    def get_next_move(self, current_position, current_direction, ir_sensors):
        for direction in self.directions:
            next_position = (current_position[0] + direction[0], current_position[1] + direction[1])
        
            if not self.visited_position(next_position):
                # Check if the move is valid using the IR sensors
                if self.is_valid_move(current_direction, direction, ir_sensors):
                    cell_next_position = self.create_cell_from_direction(self.direction_mapping[direction])
                    self.visited.add(cell_next_position)
                    self.stack.append(cell_next_position)  
                    return self.direction_mapping[direction], next_position  # Return the direction and the next position to move to
        
        # If no valid moves found, backtrack
        if self.stack:
            last_cell = self.stack.pop()  # Pop the last visited cell
            last_cell_middle_position = self.get_cell_middle_position(last_cell)
            return self.opposite_direction_mapping[current_direction], last_cell_middle_position

        return None
    

    def visited_position(self, position):
        """Check if the position (x, y) has been visited by analysing the visited cells"""
        x, y = position
        for cell in self.visited:
            (bl_x, bl_y), (tr_x, tr_y) = cell 
            if bl_x <= x <= tr_x and bl_y <= y <= tr_y:
                return True
        return False
    
    
    def is_valid_move(self, current_direction, target_direction, ir_sensors):
        
        # Define threshold for obstacle detection (adjust as needed)
        obstacle_threshold = 1.5
        
        # Determine which sensor to check based on current and target directions
        if current_direction == NORTH:
            if target_direction == MOVE_NORTH:
                return ir_sensors["center"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["back"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["left"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["right"] < obstacle_threshold

        elif current_direction == SOUTH:
            if target_direction == MOVE_NORTH:
                return ir_sensors["back"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["center"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["right"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["left"] < obstacle_threshold

        elif current_direction == EAST:
            if target_direction == MOVE_NORTH:
                return ir_sensors["left"] < obstacle_threshold
            elif target_direction == MOVE_SOUTH:
                return ir_sensors["right"] < obstacle_threshold
            elif target_direction == MOVE_WEST:
                return ir_sensors["back"] < obstacle_threshold
            elif target_direction == MOVE_EAST:
                return ir_sensors["center"] < obstacle_threshold

        elif current_direction == WEST:
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
        