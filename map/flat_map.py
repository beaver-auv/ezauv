import numpy as np
from ezauv.map.map import Map
from ezauv.utils.kalman_filter import KalmanFilter2D
from scipy.spatial.transform import Rotation as R
from ezauv.telemetry import TELEMETRY

class FlatMap(Map):    
    def __init__(self, max_velocity: float, bot_radius: float, R: np.ndarray = None, P0: np.ndarray = None, sigma_a: float=1, sigma_alpha: float=5.0):
        """The bot is assumed to start at (0, 0)"""
        
        if R is None:
            R = np.diag([1e-3, 1e-3, 0.01, 0.01, 0.01, 0.01])
        if P0 is None:
            P0 = np.diag([1.0,1.0,0.1,1.0,1.0,1.0])
        self.max_speed = max_velocity
        self.bot_radius = bot_radius
        self.velocities = np.zeros((2,))   # velocity per-axis
        self.angular_velocity = 0.0
        self.full_velocities = np.zeros((6,))  # full velocities, [x, y, z, roll, pitch, yaw]
        self.position = None
        self.heading = 0.0

        self.R = R
        self.P0 = P0
        self.sigma_a = sigma_a
        self.sigma_alpha = sigma_alpha

    def angle_to(self, heading: float) -> float:
        """Returns the current difference in heading angle to a given global angle."""
        return self.heading - heading

    def angle_to_position(self, position) -> float:
        """Returns the heading angle needed to face a given posiiton."""
        return np.atan2(position[1]-self.position[1], position[0]-self.position[0])
    
    def angle_from_position(self, position) -> float:
        """Returns the angle from a given position to the robot."""
        return np.atan2(self.position[1]-position[1], self.position[0]-position[0])

    def distance(self, position) -> float:
        """The distance from the robot to the given position."""
        return np.sqrt(np.sum(np.square(self.position-position)))
    
    def is_stopped(self, velocity_threshold=0.5) -> bool:
        """Whether the robot is currently stopped."""
        return np.all(np.isclose(self.velocities, 0, atol=velocity_threshold))

    def stopped_at(self, position, radius, velocity_threshold=0.5) -> bool:
        """Whether the robot is currently stopped within a radius of a given position."""
        distance = self.distance(position)
        return (radius >= distance) and self.is_stopped(velocity_threshold=velocity_threshold)
    
    def global_to_local_vector(self, vector: np.ndarray) -> np.ndarray:
        """Convert a global vector to a local vector based on the current heading."""
        if(len(vector) == 2):
            vector = np.array([vector[0], vector[1], 0.0])
        rot = R.from_euler('z', -self.heading)
        return rot.apply(vector)
    
    def local_to_global_vector(self, vector: np.ndarray) -> np.ndarray:
        """Convert a local vector to a global vector based on the current heading."""
        if(len(vector) == 2):
            vector = np.array([vector[0], vector[1], 0.0])
        rot = R.from_euler('z', self.heading)
        return rot.apply(vector)
    
    def update(self, sensor_data):
        super().update(sensor_data)
        if(self.position is None):
            self.position = sensor_data.get("position")
            if self.position is None:
                self.position = np.array([0.0, 0.0])
            self.P0 = self.P0
            self.R = self.R
            self.kf = KalmanFilter2D(
                H = np.eye(6),
                R = self.R,
                P0 = self.P0,
                x0 = np.array([self.position[0], self.position[1], self.heading, 0.0, 0.0, 0.0]),
                sigma_accel = self.sigma_a,
                sigma_gyro = self.sigma_alpha
            )

        dt = sensor_data.get("dt", None)
        if dt is None:
            return

        rotation = sensor_data.get("rotation").inv()
        local_accel = sensor_data.get("acceleration")

        # print("accel:", rotation.apply(local_accel))
        if rotation is not None and local_accel is not None:
            g = np.array([0.0, 0.0, 0.0]) # TODO add gravity to sim

            # body -> world
            a_world = rotation.apply(local_accel)

            # planar motion only
            ax, ay = a_world[0], a_world[1]

            self.kf.predict(
                dt=dt,
                imu_accel=(ax, ay),
            )
        else:
            # print("dt:", dt)
            self.kf.predict(
                dt=dt,
                imu_accel=(0.0, 0.0),
            )

        z = []
        H = []

        position = sensor_data.get("position", None)
        velocity = sensor_data.get("velocity", None)
        rotational_velocity = sensor_data.get("gyro", None)

        if position is not None:
            z.extend([position[0], position[1]])
            H.extend([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
            ])
            TELEMETRY.submit("position measurement x", position[0])

        if rotation is not None:
            theta = sensor_data.get("heading", None)
            z.append(theta)
            H.append([0, 0, 1, 0, 0, 0])

        if velocity is not None:
            linear_vel = velocity["translational"]
            z.extend([linear_vel[0], linear_vel[1]])
            H.extend([
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
            ])
        if rotational_velocity is not None:
            z.append(rotational_velocity)
            H.append([0, 0, 0, 0, 0, 1])
        
        if z:
            z = np.asarray(z)
            H = np.asarray(H)

            
            self.kf.update(z, H)
        x = self.kf.state()
        # k = np.array([position[0], position[1], theta, velocity["translational"][0], velocity["translational"][1], rotational_velocity])
        # print("Difference: ", np.round(x - k, 6))
        self.position = np.array([x[0], x[1]])
        self.heading = x[2]
        self.velocities = np.array([x[3], x[4]])
        self.angular_velocity = x[5]
        self.full_velocities = np.array([x[3], x[4], 0.0, 0.0, 0.0, x[5]])

        TELEMETRY.submit("estimated velocity x", self.velocities[0])
        TELEMETRY.submit("estimated velocity y", self.velocities[1])
        TELEMETRY.submit("estimated angular velocity", self.angular_velocity)
        TELEMETRY.submit("position", self.position)
        TELEMETRY.submit("rotation", self.heading)
        TELEMETRY.submit("velocity", self.velocities)
        TELEMETRY.submit("position x", self.position[0])