# TODO: get an explanation for this kalman filter or do it as Lau did

# import numpy as np


# class AngleKalmanFilter:
#     def __init__(self):
#         # State transition matrix (more conservative)
#         self.F = np.array(
#             [[1, 0.05], [0, 1]]  # Reduce velocity impact
#         )  # Add damping to acceleration

#         # Process noise covariance (increase to trust predictions less)
#         self.Q = np.array([[0.01, 0], [0, 0.02]])

#         # Measurement noise covariance (give more weight to measurements)
#         self.R = np.array([[0.1]])

#         # Initial state uncertainty
#         self.P = np.eye(2) * 500

#         self.x_pred = np.array([[0], [0]])  # Initial prediction
#         self.x_updated = np.array([[0], [0]])  # Initial update

#         self.firstTime = True

#     def predict(self, x):
#         # More conservative prediction
#         x = np.array([[x], [0]])

#         self.x_pred = self.F @ x
#         self.P = self.F @ self.P @ self.F.T + self.Q

#     def update(self, z):
#         # Measurement update with more measurement trust
#         H = np.array([[1, 0]])  # Measurement model
#         y = z - H @ self.x_pred
#         S = H @ self.P @ H.T + self.R
#         K = self.P @ H.T @ np.linalg.inv(S)
#         self.x_updated = self.x_pred + K @ y
#         self.P = (np.eye(len(self.x_pred)) - K @ H) @ self.P

#     def get_estimate(self):
#         return self.x_updated, self.x_pred

#     # function to reset the filter
#     def reset(self):
#         self.x_pred = np.array([[0], [0]])  # Initial prediction
#         self.x_updated = np.array([[0], [0]])  # Initial update
#         self.P = np.eye(2) * 500
#         self.firstTime = True


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


# TODO: new implemntation
import numpy as np


class AngleKalmanFilter:
    def __init__(self):
        # State transition matrix (more conservative)
        self.F = np.array(
            [
                [1, -0.5, 0.5],
                [0, 0.5, 0],
                [0, 0, 0.5],
            ]
        )

        self.B = np.array([[-0.5, 0.5], [0.5, 0], [0, 0.5]])

        # Process noise covariance (increase to trust predictions less)
        self.Q = np.array([[0, 0, 0]])

        # Measurement noise covariance (give more weight to measurements)
        self.R = np.array([[4]])  # 2²

        # Initial state uncertainty
        self.P = np.eye(3) * 1e-3

        self.x_pred = np.zeros((3, 1))  # Initial prediction
        self.x_updated = np.zeros((3, 1))

        self.firstTime = True

    def predict(self, x, u):
        # More conservative prediction

        # self.Q is equal to (x * 0.015)^2
        self.Q = np.array([
            [0.015**2],
            [(x[1, 0] * 0.015)**2],
            [(x[2, 0] * 0.015)**2]
        ])
        

        self.x_pred = self.F @ x + self.B @ u

        self.P = self.F @ self.P @ self.F.T + self.Q

        
    def update(self, z):
        # Measurement update with more measurement trust
        H = np.array([[1,0, 0]])  # Measurement model

        y = z - H @ self.x_pred

        S = H @ self.P @ H.T + self.R

        K = self.P @ H.T @ np.linalg.inv(S)

        self.x_updated = self.x_pred + K @ y
        self.P = (np.eye(len(self.x_pred)) - K @ H) @ self.P

    def get_estimate(self):
        return self.x_updated, self.x_pred

    # function to reset the filter
    def reset(self):
        self.P = np.eye(3) * 1e-3
        self.x_pred = np.zeros((3, 1))  # Initial prediction
        self.x_updated = np.zeros((3, 1))
        self.firstTime = True
