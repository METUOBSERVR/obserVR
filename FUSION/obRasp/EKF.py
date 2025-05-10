import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, dt):
        self.dt = dt  # Time step

        # State vector: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)

        # State covariance matrix
        self.P = np.eye(6) * 1.0

        # Process noise covariance
        q = 1e-2
        self.Q = np.diag([q, q, q, q, q, q])  # Larger noise for velocity to let it evolve

        # Measurement noise covariance (position and velocity measured)
        r_pos = 0.05  # Position measurement noise
        r_vel = 0.1   # Velocity measurement noise
        self.R = np.diag([r_pos, r_pos, r_pos, r_vel, r_vel, r_vel])

        # Measurement matrix: position and velocity are now observed
        self.H = np.zeros((6, 6))
        self.H[0, 0] = 1  # x
        self.H[1, 1] = 1  # y
        self.H[2, 2] = 1  # z
        self.H[3, 3] = 1  # vx
        self.H[4, 4] = 1  # vy
        self.H[5, 5] = 1  # vz

    def predict(self):
        # State prediction: [x, y, z, vx, vy, vz]
        F = np.eye(6)
        F[0, 3] = self.dt  # Position update equation (velocity affects position)
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        # Predict state and covariance
        self.state = np.dot(F, self.state)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update(self, position, velocity):
        # Measurement update: position and velocity (x, y, z, vx, vy, vz)
        z = np.array([position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]])

        # Innovation (difference between measurement and prediction)
        y = z - np.dot(self.H, self.state)

        # Compute Kalman gain
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update state estimate
        self.state = self.state + np.dot(K, y)

        # Update covariance
        self.P = np.dot(np.eye(6) - np.dot(K, self.H), self.P)

    def get_state(self):
        # Return the estimated position (x, y, z) and velocity (vx, vy, vz)
        return self.state[:3]  # Return position only
