import numpy as np
import matplotlib.pyplot as plt
from KalmanFilter import AngleKalmanFilter

# class AngleKalmanFilter:
#     def __init__(self):
#         # State transition matrix (more conservative)
#         self.F = np.array(
#             [
#                 [1, -0.5, 0.5],
#                 [0, 0.5, 0],
#                 [0, 0, 0.5],
#             ]
#         )

#         self.B = np.array([[-0.5, 0.5], [0.5, 0], [0, 0.5]])

#         # Process noise covariance (increase to trust predictions less)
#         self.Q = np.array([[0, 0, 0]])

#         # Measurement noise covariance (give more weight to measurements)
#         self.R = np.array([[4]])  # 2²

#         # Initial state uncertainty
#         self.P = np.eye(3) * 1e-3

#         self.x_pred = np.zeros((3, 1))  # Initial prediction
#         self.x_updated = np.zeros((3, 1))

#         self.firstTime = True

#     def predict(self, x, u):
#         # More conservative prediction

#         # self.Q is equal to (x * 0.015)^2
#         self.Q = np.array([
#             [x[0, 0]**2, 0, 0],
#             [0, (x[1, 0] * 0.015)**2, 0],
#             [0, 0, (x[2, 0] * 0.015)**2]
#         ])
        
#         self.x_pred = self.F @ x + self.B @ u

#         self.P = self.F @ self.P @ self.F.T + self.Q

#     def update(self, z):
#         # Measurement update with more measurement trust
#         H = np.array([[1,0, 0]])  # Measurement model

#         y = z - H @ self.x_pred

#         S = H @ self.P @ H.T + self.R

#         K = self.P @ H.T @ np.linalg.inv(S)

#         self.x_updated = self.x_pred + K @ y
#         self.P = (np.eye(len(self.x_pred)) - K @ H) @ self.P

#     def get_estimate(self):
#         return self.x_updated, self.x_pred

def generate_noisy_angle_data(true_angle_func, num_steps, noise_std=0.1):
    """
    Generate noisy angle measurements based on a true angle function.
    
    :param true_angle_func: Function that generates true angle at each step
    :param num_steps: Number of time steps
    :param noise_std: Standard deviation of measurement noise
    :return: Tuple of true angles and noisy measurements
    """
    true_angles = np.zeros(num_steps)
    noisy_angles = np.zeros(num_steps)
    
    for i in range(num_steps):
        true_angles[i] = true_angle_func(i)
        noisy_angles[i] = true_angles[i] + np.random.normal(0, noise_std)
    
    return true_angles, noisy_angles

def run_simulation(true_angle_func, name, noise_std=0.1):
    # Simulation parameters
    num_steps = 200
    
    # Generate noisy angle data with noise_std
    true_angles, noisy_angles = generate_noisy_angle_data(true_angle_func, num_steps, noise_std)
    
    # Initialize Kalman Filter
    kf = AngleKalmanFilter()
    
    # Arrays to store estimates
    estimated_angles = np.zeros(num_steps)
    predicted_angles = np.zeros(num_steps)
    
    # Initial state and control input
    x = np.zeros((3, 1))
    u = np.zeros((2, 1))
    
    # Run Kalman Filter
    for i in range(num_steps):
        # Update control input (you can modify this based on your specific scenario)
        u[0, 0] = 0.1 * np.sin(i * 0.1)
        u[1, 0] = 0.05 * np.cos(i * 0.1)
        
        # Predict step
        kf.predict(x, u)
        
        # Update step with noisy measurement
        z = np.array([[noisy_angles[i]]])
        kf.update(z)
        
        # Get current estimates
        x_updated, x_pred = kf.get_estimate()
        
        # Store the angle estimates
        estimated_angles[i] = x_updated[0, 0]
        predicted_angles[i] = x_pred[0, 0]
        
        # Update state for next iteration
        x = x_updated
    
    # Plotting
    plt.figure(figsize=(12, 6))
    plt.plot(true_angles, label='True Angle', color='green')
    plt.plot(noisy_angles, label='Noisy Measurements', color='red', alpha=0.5)
    plt.plot(estimated_angles, label='Kalman Filter Estimate', color='blue')
    plt.plot(predicted_angles, label='Kalman Filter Prediction', color='purple', linestyle='--')
    
    plt.title(f'Angle Kalman Filter: {name}')
    plt.xlabel('Time Step')
    plt.ylabel('Angle (degrees)')
    plt.legend()
    plt.grid(True)
    plt.show()


def main():
    # Scenario 1: Oscillating around zero with negative angles
    def scenario1(t):
        return 30 * np.sin(t * 0.1)
    
    # Scenario 2: More complex angle dynamics with negative and positive regions
    def scenario2(t):
        return 45 * np.sin(t * 0.08) - 15 * np.cos(t * 0.12)
    
    # Scenario 3: Sudden jumps and negative angles
    def scenario3(t):
        if t < 50:
            return -20 * np.sin(t * 0.1)
        elif t < 100:
            return 25 * np.sin((t-50) * 0.1)
        else:
            return -35 * np.sin((t-100) * 0.1)

    def zero_to_chaos(t):
        # Start very close to zero
        if t < 50:
            return 0.1 * t * np.sin(t * 0.1)
        
        # Gradually increase amplitude and complexity
        elif t < 100:
            return 15 * np.sin(t * 0.2) * np.cos(t * 0.15)
        
        # Full chaotic oscillation
        elif t < 200:
            return 45 * np.sin(t * 0.3) * np.tan(t * 0.1) + 20 * np.cos(t * 0.25)
        
        # Even more extreme oscillations
        else:
            return 80 * np.sin(t * 0.4) * np.exp(np.sin(t * 0.1)) - 30 * np.cos(t * 0.2)
    
    # Run simulations
    print("Running Scenario 1: Simple Negative Angle Oscillation")
    run_simulation(scenario1, "Negative Angle Oscillation")
    
    print("Running Scenario 2: Complex Angle Dynamics")
    run_simulation(scenario2, "Complex Angle Dynamics")
    
    print("Running Scenario 3: Angle with Sudden Jumps")
    run_simulation(scenario3, "Angle with Sudden Jumps")

    print("Running Scenario: Zero to Chaos")
    run_simulation(zero_to_chaos, "Zero to Chaos Angle Dynamics", noise_std=1.0)

if __name__ == '__main__':
    main()
