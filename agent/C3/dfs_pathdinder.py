MOVE_NORTH = (0, 2)
MOVE_SOUTH = (0, -2)
MOVE_WEST = (-2, 0)
MOVE_EAST = (2, 0)

NORTH = 90.0
SOUTH = -90.0
WEST = 180.0
EAST = 0.0

class DFSPathfinder:
    def __init__(self):
        self.visited = set()  # Set of visited cells
        self.stack = []  # Stack to keep track of DFS path
        self.directions = [NORTH, EAST, SOUTH, WEST] # Direction search priority
        
        self.directions_to_vector = {
            NORTH: MOVE_NORTH,
            EAST: MOVE_EAST,
            SOUTH: MOVE_SOUTH,
            WEST: MOVE_WEST
        }  

        self.opposite_directions_mapping = {
            NORTH: SOUTH,
            EAST: WEST,
            SOUTH: NORTH,
            WEST: EAST
        }


    def initialize(self, initial_position):
        """Initialize DFS with the robot's starting position."""
        bottom_left = (initial_position[0] - 1, initial_position[1] - 1)
        top_right = (initial_position[0] + 1, initial_position[1] + 1)
        cell = Cell(bottom_left, top_right)
        self.stack.append(cell)
        self.visited.add(cell)


    def get_next_move(self, current_position, current_direction, ir_sensors):
        for direction in self.directions:
            move_vector = self.directions_to_vector[direction]
            next_position = (current_position[0] + move_vector[0], current_position[1] + move_vector[1])
        
            if not self.visited_position(next_position):
                # Check if the move is valid using the IR sensors
                if self.is_valid_move(current_direction, direction, ir_sensors):
                    cell_next_position = self.create_cell_from_direction(direction)
                    self.visited.add(cell_next_position)
                    self.stack.append(cell_next_position)  
                    return direction, next_position, cell_next_position  # Return the direction and the next position to move to
        
        # If no valid moves found, backtrack
        if self.stack:
            self.stack.pop() # Pop the current cell robot is on
            last_cell = self.stack.pop() # Pop the last cell the robot has been
            last_cell_middle_position = last_cell.get_cell_middle_position()
            return self.opposite_directions_mapping[current_direction], last_cell_middle_position, last_cell

        return None
    

    def visited_position(self, position):
        """Check if the position (x, y) has been visited by analysing the visited cells"""
        x, y = position
        for cell in self.visited:
            (bl_x, bl_y), (tr_x, tr_y) = cell.coordinates
            if bl_x <= x <= tr_x and bl_y <= y <= tr_y:
                return True
        return False
    
    
    def create_cell_from_direction(self, direction):
        """Create a cell based on the robot's upcoming direction."""

        last_cell = self.stack[-1]  # Get the last visited cell
        (bl_x, bl_y), (tr_x, tr_y) = last_cell.coordinates  # Unpack the last cell coordinates

        if direction == NORTH: 
            bottom_left = (bl_x, tr_y)
            top_right = (tr_x, tr_y + 2)
        elif direction == SOUTH: 
            bottom_left = (bl_x, bl_y - 2)
            top_right = (tr_x, bl_y)
        elif direction == WEST: 
            bottom_left = (bl_x - 2, bl_y)
            top_right = (bl_x, tr_y)
        elif direction == EAST: 
            bottom_left = (tr_x, bl_y)
            top_right = (tr_x + 2, tr_y)
        else: 
            raise ValueError(f"Invalid direction: {direction}")
            
        # Return the cell as a tuple of bottom-left and top-right corners
        return Cell(bottom_left, top_right)
    
    
    def is_valid_move(self, current_direction, target_direction, ir_sensors):
        
        # Define threshold for obstacle detection (adjust as needed)
        obstacle_threshold = 1.5
        
        sensor_map = {
            NORTH: {
                NORTH: "center",
                SOUTH: "back",
                WEST: "left",
                EAST: "right"
            },
            SOUTH: {
                NORTH: "back",
                SOUTH: "center",
                WEST: "right",
                EAST: "left"
            },
            EAST: {
                NORTH: "left",
                SOUTH: "right",
                WEST: "back",
                EAST: "center"
            },
            WEST: {
                NORTH: "right",
                SOUTH: "left",
                WEST: "center",
                EAST: "back"
            }
        }

        sensor = sensor_map.get(current_direction, {}).get(target_direction)
        
        if sensor:
            return ir_sensors[sensor] <= obstacle_threshold
        
        return False

        
class Cell:
    def __init__(self, bottom_left, top_right):
        self.coordinates = (bottom_left, top_right)
        self.bl_x, self.bl_y = bottom_left
        self.tr_x, self.tr_y = top_right

    def get_cell_middle_position(self):
        """Calculate the middle position of a given cell."""
        return (self.tr_x - 1, self.tr_y - 1)

        
    

