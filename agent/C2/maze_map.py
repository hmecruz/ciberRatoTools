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
    
    @staticmethod
    def inside_cell(position: tuple, cell):
        x, y = position
        (bl_x, bl_y), (tr_x, tr_y) = cell.coordinates
        return (bl_x <= x < tr_x) and (bl_y <= y < tr_y)