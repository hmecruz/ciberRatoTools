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
        self.R = np.array([[0.000030864]])  # 2² or (2/360)² = 0.000030864

        # Initial state uncertainty
        self.P = np.eye(3) * 1e-3

        self.x_pred = np.zeros((3, 1))  # Initial prediction
        self.x_updated = np.zeros((3, 1))

        self.firstTime = True

    def predict(self, x, u):
        # More conservative prediction

        # self.Q is equal to (x * 0.015)^2
        self.Q = np.array([
            [0,0,0],
            [0,(x[1, 0] * 0.015)**2,0],
            [0,0,(x[2, 0] * 0.015)**2]
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
