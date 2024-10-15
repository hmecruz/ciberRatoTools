from collections import deque

class NoiseFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)  # Use deque with maxlen for automatic management

    def update(self, current_value):
        self.values.append(current_value)
        filtered_value = sum(self.values) / len(self.values)

        return round(filtered_value, 1)