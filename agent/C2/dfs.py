
"""DFS with Backtracking"""
class DFS:
    def __init__(self, start):
        self.start = start  # Starting position of the robot (x, y)
        self.visited = set()  # Set to keep track of visited cells
        self.stack = []  # Stack for DFS

        self.directions = ["E", "W", "N", "S"]

    def is_valid(self, x, y):
        """Check if a cell (x, y) is within bounds and not visited or blocked."""
        rows, cols = len(self.grid), len(self.grid[0])
        return 0 <= x < rows and 0 <= y < cols and (x, y) not in self.visited and self.grid[x][y] == 0

    def dfs(self):
        """Perform DFS iteratively using a stack."""
        self.stack.append(self.start)
        self.visited.add(self.start)

        while self.stack:
            current = self.stack.pop()
            x, y = current

            # Perform any necessary actions at the current position
            print(f"Visiting cell: ({x}, {y})")

            # Explore neighbors (up, down, left, right)
            for direction in self.directions:
                next_x, next_y = x + direction[0], y + direction[1]
                
                # If the next cell is valid and not visited
                if self.is_valid(next_x, next_y):
                    self.stack.append((next_x, next_y))
                    self.visited.add((next_x, next_y))