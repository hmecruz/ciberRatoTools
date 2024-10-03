from collections import deque

class NoiseFilter:
    def __init__(self, window_size=5, noise_threshold=0.1):
        self.window_size = window_size
        self.noise_threshold = noise_threshold
        self.values = deque(maxlen=window_size)  # Use deque with maxlen for automatic management

    def update(self, current_value):
        self.values.append(current_value)
        filtered_value = sum(self.values) / len(self.values)
        
        # Compute the min and max bounds based on noise threshold
        min_filtered_value = current_value * (1 - self.noise_threshold)
        max_filtered_value = current_value * (1 + self.noise_threshold)

        filtered_value = max(min_filtered_value, min(max_filtered_value, filtered_value))

        return round(filtered_value, 1)