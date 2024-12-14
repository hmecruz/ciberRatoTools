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


SENSOR_THRESHOLD = 1.1 # Wall Detection

CENTER_SENSOR_SETPOINT = 10 # (1/0.2)

# Reliability Threshold
RELIABILITY_THRESHOLD_RECALIBRATION = 0.25

# Velocity limits
MAX_POW = 0.15
MIN_POW = -0.15

# Sampling Time
TIME_STEP = 0.005

# Throttle PD Controller Values
KP = 0.35 
KD = 0 

# Speed Steering PD Controller Values
KPSS = 0.1
KDSS = 0.01

# Steering PD Controller Values
KPS = 0.02 
KDS = 0.00003 

# Recalibration PD Controller Values
KPR = 0.008
KDR = 0

# Maze Size
CELLROWS=7
CELLCOLS=14

# Recalibration after n moves
RECALIBRATION_PERIOD_X = 30
RECALIBRATION_PERIOD_Y = 30


def vector_to_direction(vector):
    vector_to_direction = {
        MOVE_NORTH: NORTH,
        MOVE_WEST: WEST,
        MOVE_EAST: EAST,
        MOVE_SOUTH: SOUTH
    }  
    direction = vector_to_direction.get(vector, None)
    if direction is None: # Keep moving forward 
        x, y = vector
        # Determine the target direction
        if x == 0 and y > 0:
            direction = NORTH  # NORTH
        elif x == 0 and y < 0:
            direction = SOUTH  # SOUTH
        elif x > 0 and y == 0:
            direction = EAST  # EAST
        elif x < 0 and y == 0:
            direction = WEST  # WEST 
        else:
            direction = None
    return direction
    

def closest_direction(angle):
    """Returns the closest cardinal direction (NORTH, SOUTH, WEST, EAST) based on the given angle."""
    # Normalize the angle to be within 0 to 360 degrees
    angle = angle % 360

    # Define the midpoints between the cardinal directions
    midpoints = {
        (45, 135): NORTH,
        (135, 225): WEST,
        (225, 315): SOUTH,
        (315, 360): EAST,
        (0, 45): EAST
    }

    # Find the closest direction
    for (start, end), direction in midpoints.items():
        if start <= angle < end:
            return direction

    # If the angle is exactly 360, it should be EAST
    return EAST
