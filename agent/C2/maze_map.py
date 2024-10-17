from constants import *

class MazeMap:
    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self.map = self._create_map()

    def _create_map(self):
        """Creates a 4x larger map representation with the robot starting at the center (0, 0)"""        
        maze_map = {}
        for row in range(-self.rows * 2 + 2, self.rows * 2 - 1, 2):
            for col in range(-self.cols * 2 + 2, self.cols * 2 - 1, 2): 
                maze_map[(col, row)] = None
        return maze_map
    
    
    def print_map(self):
        """Prints the map with ' ' for None and 'X' for cells."""
        for row in range(-self.rows * 2 + 2, self.rows * 2 - 1, 2):
            row_str = ""
            for col in range(-self.cols * 2 + 2, self.cols * 2 - 1, 2):
                if self.map[(col, row)] is None:
                    row_str += " "
                else:
                    row_str += "X"
            print(row_str)

  
    def add_cell_map(self, cell, index: tuple):
        """Add cell to map"""
        self.map[(index)] = cell
        cell.mark_visited()

    
    def get_cell(self, coordinates: tuple):
        x, y = coordinates
        # Loop through all cells and find the one containing the given coordinates
        for cell in self.map.values():
            if cell is None: continue
            bl_x, bl_y = cell.coordinates[0]
            tr_x, tr_y = cell.coordinates[1]
            # Check if coordinates are within the bounds of this cell
            if bl_x <= x <= tr_x and bl_y <= y <= tr_y:
                return cell
        return None
    
    
    def get_visited_cells(self):
        """Returns a list of all visited cells."""
        visited_cells = [cell for cell in self.map.values() if cell and cell.is_visited()]
        return visited_cells
    

    def has_neighbours_to_explore(self, cell):
        """Check if the given cell has neighbours to explore."""
        cell_middle_position = cell.get_middle_position() 
        vectors = [MOVE_NORTH, MOVE_WEST, MOVE_EAST, MOVE_SOUTH]
        
        for vector in vectors:
            neighbour_coords = (
                cell_middle_position[0] + vector[0],
                cell_middle_position[1] + vector[1]
            )
            if not self.get_cell(neighbour_coords): # If neighbour cell not in map
                if getattr(cell, cell.vector_wall(vector)) == False: # If no wall
                    return True
        return False
    

    def get_neighbours(self, cell):
        """Return a list of neighbour cells"""
        neighbour_cells = []

        cell_middle_position = cell.get_middle_position() 
        vectors = [MOVE_NORTH, MOVE_WEST, MOVE_EAST, MOVE_SOUTH]
        
        for vector in vectors:
            neighbour_coords = (
                cell_middle_position[0] + vector[0],
                cell_middle_position[1] + vector[1]
            )

            neighbour_cell = self.get_cell(neighbour_coords) # If neighbour cell in map
            if neighbour_cell is not None: 
                neighbour_cells.append(neighbour_cell)
        
        return neighbour_cells


    def mark_cell_visited(self, coordinates: tuple):
        """Marks the cell at the given coordinates as visited."""
        cell = self.get_cell(coordinates)
        if cell:
            cell.mark_visited()


    def is_cell_visited(self, coordinates: tuple):
        """Checks if the cell at the given coordinates is visited."""
        cell = self.get_cell(coordinates)
        if cell:
            return cell.is_visited()
        return False
    

class Cell:
    def __init__(self, bottom_left: tuple):
        self.bl_x, self.bl_y = bottom_left
        self.tr_x, self.tr_y = self.bl_x + 2, self.bl_y + 2
        self.coordinates = ((self.bl_x, self.bl_y), (self.tr_x, self.tr_y))
        self.visited = False

        self.left_wall = False
        self.right_wall = False
        self.top_wall = False
        self.bottom_wall = False


    def get_middle_position(self):
        """Calculate the middle position of the cell."""
        return (self.bl_x + 1, self.bl_y + 1)
    

    def mark_visited(self):
        """Mark the cell as visited."""
        self.visited = True


    def is_visited(self):
        """Check if the cell is visited."""
        return self.visited
    

    def mark_walls(self, ir_sensors, current_direction):
        # Define the sensor directions for each heading (NORTH, SOUTH, EAST, WEST)
        sensor_map = {
            NORTH: {
                "center": "top_wall",
                "left": "left_wall",
                "right": "right_wall",
                "back": "bottom_wall",
            },
            WEST: {
                "center": "left_wall",
                "left": "bottom_wall",
                "right": "top_wall",
                "back": "right_wall",
            },
            EAST: {
                "center": "right_wall",
                "left": "top_wall",
                "right": "bottom_wall",
                "back": "left_wall",
            },
            SOUTH: {
                "center": "bottom_wall",
                "left": "right_wall",
                "right": "left_wall",
                "back": "top_wall",
            },
        }
    
        for sensor_name, sensor_value in ir_sensors.items():
            if sensor_value > SENSOR_THRESHOLD:
                wall_to_mark = sensor_map[current_direction].get(sensor_name)
                if wall_to_mark:
                    setattr(self, wall_to_mark, True)  # Mark the wall as True
    
    
    def vector_wall(self, vector):
        """Detects if a cell has a wall based on a vector"""
        if vector == MOVE_NORTH:
            return 'top_wall'
        elif vector == MOVE_WEST:
            return 'left_wall'
        elif vector == MOVE_EAST:
            return 'right_wall'
        elif vector == MOVE_SOUTH:
            return 'bottom_wall'
        
    @staticmethod
    def inside_cell(position: tuple, cell):
        x, y = position
        (bl_x, bl_y), (tr_x, tr_y) = cell.coordinates
        return (bl_x <= x < tr_x) and (bl_y <= y < tr_y)