# import numpy as np

# class AngleKalmanFilter:
#     def __init__(self, initial_state, process_variance, measurement_variance):
#         # Initial state (angle)
#         self.state_estimate = initial_state  # Initial guess (initial compass uncertainty)
#         self.estimate_covariance = 0.2  # Initial covariance (uncertainty in estimate)

#         # Noise parameters
#         self.process_variance = process_variance  # Noise from the motion model 
#         self.measurement_variance = measurement_variance  # Noise from the compass sensor
#         self.firstTime = True

#     def predict(self, control_input):
#         # Prediction step: Update state estimate based on control input (motion model)
#         # control_input is the rotational velocity (change in angle)
#         self.state_estimate += control_input
#         self.estimate_covariance += self.process_variance  # Increase uncertainty due to process noise

#     def update(self, measurement):
#         # Update step: Incorporate sensor measurement
#         kalman_gain = self.estimate_covariance / (
#             self.estimate_covariance + self.measurement_variance
#         )

#         # Correct state estimate
#         self.state_estimate = (
#             self.state_estimate
#             + kalman_gain * (measurement - self.state_estimate)
#         )

#         # Update estimate covariance
#         self.estimate_covariance = (1 - kalman_gain) * self.estimate_covariance

#     def get_estimate(self):
#         # Normalize angle to [-180, 180]
#         # normalized_angle = (self.state_estimate + 180) % 360 - 180
#         # return normalized_angle
#         return self.state_estimate

# ----

# import numpy as np

# class AngleKalmanFilter:
#     def __init__(self, initial_angle=0.0, initial_uncertainty=1.0):
#         # State initialization
#         self.x = np.array([[initial_angle]])  # State (angle in degrees)
#         self.P = np.array([[initial_uncertainty]])  # State uncertainty
        
#         # Measurement noise (compass noise)
#         self.R = np.array([[0.005555556]])  # Square of noise std for variance (in degrees)
        
#         # Process noise (model uncertainty)
#         self.Q = np.array([[0.1]])  # Can be tuned based on movement model uncertainty (in degrees)
        
#         # Measurement matrix
#         self.H = np.array([[1.0]])  # Direct measurement of angle
        
#         # State transition matrix
#         self.F = np.array([[1.0]])  # Identity for angle

#         self.firstTime = True
        
#     def _normalize_angle_degrees(self, angle):
#         """Normalize angle to [-180, 180] degrees"""
#         return (angle + 180) % 360 - 180
        
#     def predict(self, movement_angle_change=0.0):
#         """
#         Predict step using movement model
#         Args:
#             movement_angle_change: Change in angle from movement model (in degrees)
#         """
#         # State transition
#         self.x = self.F @ self.x + movement_angle_change
#         self.x[0,0] = self._normalize_angle_degrees(self.x[0,0])
        
#         # Covariance prediction
#         self.P = self.F @ self.P @ self.F.T + self.Q
        
#     def update(self, compass_measurement):
#         """
#         Update step using compass measurement
#         Args:
#             compass_measurement: Angle measurement from compass (in degrees)
#         """
#         # Convert measurement to numpy array if it isn't already
#         measurement = np.array([[compass_measurement]])
        
#         # Normalize angle difference
#         innovation = measurement - self.H @ self.x
#         innovation[0,0] = self._normalize_angle_degrees(innovation[0,0])
        
#         # Kalman gain
#         S = self.H @ self.P @ self.H.T + self.R
#         K = self.P @ self.H.T @ np.linalg.inv(S)
        
#         # State update
#         self.x = self.x + K @ innovation
#         self.x[0,0] = self._normalize_angle_degrees(self.x[0,0])
        
#         # Covariance update
#         self.P = (np.eye(1) - K @ self.H) @ self.P
        
#     def get_estimate(self):
#         """Return current estimate and uncertainty"""
#         return float(self.x[0,0]), float(self.P[0,0])
    

# TODO: get an explanation for this kalman filter or do it as Lau did

import numpy as np

class AngleKalmanFilter:
    def __init__(self):
        # State transition matrix (more conservative)
        self.F = np.array([[1, 0.1],   # Reduce velocity impact
                           [0, 1]])  # Add damping to acceleration
        
        # Process noise covariance (increase to trust predictions less)
        self.Q = np.array([[0.01, 0],
                           [0, 0.02]])
        
        # Measurement noise covariance (give more weight to measurements)
        self.R = np.array([[0.1]])
        
        # Initial state uncertainty
        self.P = np.eye(2) * 500

        self.x_pred = np.array([[0], [0]])  # Initial prediction
        self.x_updated = np.array([[0], [0]])  # Initial update

        self.firstTime = True

    def predict(self, x):
        # More conservative prediction
        x = np.array([[x], [0]])

        self.x_pred = self.F @ x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        # Measurement update with more measurement trust
        H = np.array([[1, 0]])  # Measurement model
        y = z - H @ self.x_pred
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x_updated = self.x_pred + K @ y
        self.P = (np.eye(len(self.x_pred)) - K @ H) @ self.P
    
    def get_estimate(self):
        return self.x_updated,self.x_pred



# ----
# Lau implementation
# import numpy as np

# # Initialize state, covariance, and matrices
# X = np.array([[0], [0]])  # Initial state [angle; angular rate]
# P = np.eye(2)             # Initial covariance
# F = np.array([[1, 0.1],   # State transition matrix (example)
#               [0, 1]])
# B = np.array([[0], [0]])  # Control input matrix (if no control input, keep as zeros)
# H = np.array([[1, 0]])    # Measurement matrix
# Q = np.array([[1e-4, 0],  # Process noise covariance
#               [0, 1e-4]])
# R = np.array([[1e-2]])    # Measurement noise covariance
# I = np.eye(2)             # Identity matrix

# # Simulated measurements (replace with your compass data)
# measurements = [30 + np.random.normal(0, 0.1) for _ in range(100)]  # Noisy measurements

# # Kalman Filter loop
# for Z in measurements:
#     Z = np.array([[Z]])  # Current measurement as column vector

#     # Prediction Step
#     X_pred = F @ X + B @ np.array([[0]])  # Replace with control input if applicable
#     P_pred = F @ P @ F.T + Q

#     # Update Step
#     K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)  # Kalman gain
#     X = X_pred + K @ (Z - H @ X_pred)                      # State update
#     P = (I - K @ H) @ P_pred                               # Covariance update

#     # Output the filtered state (angle and angular rate)
#     print(f"Filtered Angle: {X[0, 0]:.2f}, Filtered Rate: {X[1, 0]:.2f}")
