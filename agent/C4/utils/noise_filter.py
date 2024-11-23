from collections import deque

NOISE = 0.1


class NoiseFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(
            maxlen=window_size
        )  # Use deque with maxlen for automatic management

    def update(self, current_value):
        def lpf(current_value: float, last_value: float):

            filtered = 0.8 * current_value + (1 - 0.8) * last_value
            print(f"CV: {current_value}\tF: {filtered}",flush=True)

            if filtered > (current_value * (1 + NOISE)):
                return current_value * (1 + NOISE)
            elif filtered < (current_value * (1 - NOISE)):
                return current_value * (1 - NOISE)
            else:
                return filtered
        
        if len(self.values) <= self.window_size - 1:
            self.values.append(current_value)
            return current_value
            

        filtered = lpf(current_value, self.values[-1])
        self.values.append(filtered)

        return filtered

    # def update(self,current_value):
    #     self.values.append(current_value)
    #     filtered_value = sum(self.values) / len(self.values)
    #     return round(filtered_value,1)
