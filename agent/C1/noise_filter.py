from collections import deque

class NoiseFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)  # Use deque with maxlen for automatic management

    def update(self, current_value):
        def lpf(current_value: float, last_value: float):
            return 0.8 * current_value + (1 - 0.8) * last_value
        if len(self.values) <= 0:
            return current_value
        filtered = lpf(current_value,self.values[-1])
        self.values.append(filtered)
        return filtered