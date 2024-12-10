from constants import *

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
    

    def mark_walls(self, ir_sensors, current_direction) -> dict[float, dict[str, str]]:
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
        
        return sensor_map[current_direction]

    
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


class MazeMap:
    def __init__(self, rows: int, cols: int, outfile: str = "maze.map"):
        self.rows = rows
        self.cols = cols
        self.cell_size = 2 # 2 coordinates
        self.map = self._create_map()
        self.robot_initial_position = (cols*2, rows*2) # Normalize initial robot position
        self.outfile = outfile

        
    def _create_map(self):
        """Creates a 4x larger map representation with the robot starting at the center (0, 0)"""        
        maze_map = {}
        for row in range(0, self.rows * self.cell_size * 2  - 1, 2):
            for col in range(0, self.cols * self.cell_size * 2 - 1, 2): 
                maze_map[(col, row)] = None # col --> x | row --> y
        return maze_map
    
    
    def print_map(self, beacons: dict[int, Cell]):
        map_representation = [[" " for _ in range(self.cols * self.cell_size * 2 - 1)] for _ in range(self.rows * self.cell_size * 2 - 1)]
    
        # map[Linhas][Colunas]
        for pos, cell in self.map.items():
            col, row = pos # col --> x, linha --> y 
            if cell is None: continue
            #print(pos)
            #print(f"({col}, {row})") # x, y
            #print(cell.top_wall)
            #print(cell.left_wall)
            #print(cell.right_wall)
            #print(cell.bottom_wall)

            map_representation[row - 1][col - 1] = "X"
            map_representation[row - 1][col] = "|" if cell.right_wall else "X"
            map_representation[row - 1][col - 2] = "|" if cell.left_wall else "X"
            map_representation[row - 2][col - 1] = "-" if cell.top_wall else "X"
            map_representation[row][col - 1] = "-" if cell.bottom_wall else "X"
        
        for id, cell in beacons.items():
            col, row = cell.get_middle_position()
            map_representation[-row][col - 1] = str(int(id))

        map_representation[self.rows*self.cell_size - 1][self.cols*self.cell_size - 1] = "0"

        # Write the final map to a text file
        with open(self.outfile, "w") as file:
            for line in map_representation:
                file.write("".join(line) + "\n")
    

    def add_cell_map(self, cell, index: tuple):
        """Add cell to map"""
        index = (
            index[0] + self.robot_initial_position[0],
            -index[1] + self.robot_initial_position[1]
        )
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
    
    def get_cell_index(self, cell: Cell):
        for key, value in self.map.items():
            if value is None: continue
            if cell == value:
                return key
        return None


    def get_visited_cells(self):
        """Returns a list of all visited cells."""
        visited_cells = [cell for cell in self.map.values() if cell and cell.is_visited()]
        return visited_cells
    
    
    def has_neighbours_to_explore(self, cell: Cell):
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
    
    
    def get_neighbours(self, cell: Cell):
        """Return a list of known neighbour cells"""
        neighbour_cells = []

        cell_middle_position = cell.get_middle_position() 
        vectors = [MOVE_NORTH, MOVE_WEST, MOVE_EAST, MOVE_SOUTH]
        
        for vector in vectors:
            neighbour_coords = (
                cell_middle_position[0] + vector[0],
                cell_middle_position[1] + vector[1]
            )

            neighbour_cell = self.get_cell(neighbour_coords)
            if neighbour_cell is not None: 
                if getattr(cell, cell.vector_wall(vector)) == False: # If no wall
                    neighbour_cells.append(neighbour_cell)
        
        return neighbour_cells
    

    def get_all_neighbours(self, cell:Cell):
        """
        Return a list of unkown and known neighbour cells
        Unknown cells are treated as passages
        Unknown cells register wall info from adjacent known cells
        """
        neighbour_cells = []

        cell_middle_position = cell.get_middle_position() 
        vectors = [MOVE_NORTH, MOVE_WEST, MOVE_EAST, MOVE_SOUTH]
        
        for vector in vectors:
            neighbour_coords = (
                cell_middle_position[0] + vector[0],
                cell_middle_position[1] + vector[1]
            )

            neighbour_cell = self.get_cell(neighbour_coords)
            if neighbour_cell is not None: 
                if getattr(cell, cell.vector_wall(vector)) == False: # If no wall
                    neighbour_cells.append(neighbour_cell)
            else: 
                if getattr(cell, cell.vector_wall(vector)) == False: # If no wall
                    fake_cell = Cell((neighbour_coords[0] - 1, neighbour_coords[1] - 1)) # Cell Bottom Left 
                    fake_cell_neighbours = self.get_neighbours(fake_cell)
                    for neighbour in fake_cell_neighbours:
                        # Mark fake_cell walls based on known cells
                        vector = (
                        neighbour.bl_x - fake_cell.bl_x,
                        neighbour.bl_y - fake_cell.bl_y
                        )
                        if vector == MOVE_NORTH and neighbour.bottom_wall:
                            fake_cell.top_wall = True
                        elif vector == MOVE_SOUTH and neighbour.top_wall:
                            fake_cell.bottom_wall = True
                        elif vector == MOVE_WEST and neighbour.right_wall:
                            fake_cell.left_wall = True
                        elif vector == MOVE_EAST and neighbour.left_wall:
                            fake_cell.right_wall = True

                    neighbour_cells.append(fake_cell)

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
    

