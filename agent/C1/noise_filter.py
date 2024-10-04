from collections import deque

class NoiseFilter:
    def __init__(self, window_size, noise_threshold):
        self.window_size = window_size
        self.noise_threshold = noise_threshold
        self.values = deque(maxlen=window_size)  # Use deque with maxlen for automatic management

    def update(self, current_value):
        self.values.append(current_value)
        filtered_value = sum(self.values) / len(self.values)
        
        # Compute the min and max bounds based on noise threshold
        
        #min_filtered_value = current_value * (1 - self.noise_threshold)
        #max_filtered_value = current_value * (1 + self.noise_threshold)
        
        #if not (min_filtered_value <= filtered_value <= max_filtered_value):
        #    filtered_value = current_value
        
        return round(filtered_value, 1)