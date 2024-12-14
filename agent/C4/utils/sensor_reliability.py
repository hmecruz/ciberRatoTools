from collections import deque
import math

class SensorReliabilty:
    def __init__(self, window_size, reliability_threshold: float):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        self.reliability_threshold = reliability_threshold

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

        if dev > self.reliability_threshold:
            return False, -1
        
        return True, round(value, 3)

    def clear_window(self):
        self.values.clear()



from collections import deque, Counter
import math

class GroundSensorReliabilty:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, current_value):
        self.values.append(current_value)

        # If the window is not full, we can't reliably detect a beacon yet
        if len(self.values) < self.window_size:
            return False, -1

        # Count occurrences of each value in the window
        count = Counter(self.values)

        # Find the most common value and its frequency
        most_common_value, frequency = count.most_common(1)[0]

        # Check if the most common value is a valid beacon ID (>= 0)
        if most_common_value >= 0:
            return True, most_common_value  # Valid beacon detected
        else:
            return False, -1  # No valid beacon detected

    def clear_window(self):
        self.values.clear()

