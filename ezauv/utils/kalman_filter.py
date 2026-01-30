import numpy as np



class KalmanFilter:
    def __init__(self, H, R, P0, x0):
        self.H_default = H
        self.R_default = R
        self.P = P0.copy()
        self.x = x0.copy()

    def predict(self, A, B, u, Q):
        """Predict step"""
        # Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.1])
        self.x = A @ self.x + B @ u
        self.P = A @ self.P @ A.T + Q

    def update(self, z, H=None):
        """Update step with arbitrary subset of measurements"""
        if H is None:
            H = self.H_default
            R = self.R_default
        else:
            H = np.asarray(H)
            R = np.zeros((H.shape[0], H.shape[0]))
            for i, row in enumerate(H):
                idxs = np.where(row != 0)[0]
                if len(idxs) == 1:
                    R[i, i] = self.R_default[idxs[0], idxs[0]]
                else:
                    R[i, i] = 1e-6  # fallback small noise

        z = np.asarray(z)
        y = z - H @ self.x  # residual

        theta_idx = None
        for i, row in enumerate(H):
            if row[2] == 1:  # maps to theta in state
                theta_idx = i
                break
        if theta_idx is not None:
            y[theta_idx] = (y[theta_idx] + np.pi) % (2*np.pi) - np.pi

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.pinv(S)

        self.x = self.x + K @ y
        self.x[2] = (self.x[2] + np.pi) % (2*np.pi) - np.pi

        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P

    def state(self):
        return self.x



class KalmanFilter2D:

    def __init__(self, H, R, P0, x0,
                 sigma_accel, sigma_gyro):
        self.kf = KalmanFilter(H, R, P0, x0)

        self.sigma_accel = sigma_accel
        self.sigma_gyro = sigma_gyro

    def build_matrices(self, dt):
        A = np.eye(6)
        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt   # theta += omega*dt

        B = np.zeros((6, 2))
        B[0, 0] = 0.5 * dt**2
        B[1, 1] = 0.5 * dt**2
        B[3, 0] = dt
        B[4, 1] = dt

        Q = np.zeros((6, 6))

        sa2 = self.sigma_accel**2
        sg2 = self.sigma_gyro**2

        Q[0, 0] = 0.25 * dt**4 * sa2
        Q[1, 1] = 0.25 * dt**4 * sa2
        Q[3, 3] = dt**2 * sa2
        Q[4, 4] = dt**2 * sa2

        Q[5, 5] = dt * sg2
        Q[2, 2] = dt * (0.1 * sg2)
        
        return A, B, Q


    def predict(self, dt, imu_accel):
        A, B, Q = self.build_matrices(dt)
        u = np.array([imu_accel[0], imu_accel[1]])
        self.kf.predict(A, B, u, Q)

        self.kf.x[2] = (self.kf.x[2] + np.pi) % (2*np.pi) - np.pi


    def update(self, z, H=None):
        self.kf.update(z, H)

    def state(self):
        return self.kf.state()