# Directions
NORTH = 90.0
EAST = 0.0
WEST = 180.0
SOUTH = -90.0

# Move Vectors
MOVE_NORTH = (0, 2)
MOVE_WEST = (-2, 0)
MOVE_EAST = (2, 0)
MOVE_SOUTH = (0, -2)


SENSOR_THRESHOLD = 1.0 # Wall Detection

# Velocity limits
MAX_POW = 0.15
MIN_POW = -0.15

# Sampling Time
TIME_STEP = 0.005

# Throttle PD Controller values
KP = 0.35 
KD = 0 

# Steering PD Controller Values
KPS = 0.02 
KDS = 0.00003 

# Maze Size
CELLROWS=7
CELLCOLS=14


def vector_to_direction(vector):
        vector_to_direction = {
            MOVE_NORTH: NORTH,
            MOVE_WEST: WEST,
            MOVE_EAST: EAST,
            MOVE_SOUTH: SOUTH
        }  
        return vector_to_direction.get(vector, None)