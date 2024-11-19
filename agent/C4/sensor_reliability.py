from collections import deque

class SensorReliabilty():
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(
            maxlen=window_size
        )
    
    def update(self, current_value):
        def checkReliability(values:list):
            avg = sum(values) / len(values)
            # make the standard deviation of list values

        self.values.append(current_value)