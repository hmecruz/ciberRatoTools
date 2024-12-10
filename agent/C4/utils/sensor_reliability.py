from collections import deque
import math

RELIABILITY_THRESHOLD = 0.27


class SensorReliabilty:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, current_value):
        def checkReliability(values: list):
            mean = sum(values) / len(values)
            squared_diff_sum = sum((x - mean) ** 2 for x in self.values)
            std_dev = (
                math.sqrt(squared_diff_sum / len(self.values))
                if len(self.values) > 1
                else 0
            )
            return mean, std_dev

        self.values.append(current_value)

        if len(self.values) < self.window_size:
            return False, -1

        value, dev = checkReliability(self.values)

        if dev > RELIABILITY_THRESHOLD:
            return False, -1
        
        return True, round(value, 3)

    def clear_window(self):
        self.values.clear()

