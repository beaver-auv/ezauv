from ezauv.hardware.sensor_interface import Sensor
from scipy.spatial.transform import Rotation as R
import numpy as np

# a class to provide fake sensor data for the simulation

def std(std_dev: float, size: int = 1) -> np.ndarray:
    """Generate random noise based on a standard deviation."""
    return np.random.normal(0, std_dev, size)

class FakeIMU(Sensor):
    """
    A simulated IMU to provide data in the correct format from the simulation.
    """

    def __init__(self, heading_std: float, acceleration_std: float, angular_acceleration_std: float, angular_velocity_std: float, acceleration_function: callable, angular_acceleration_function: callable, angular_velocity_function: callable, heading_function: callable):
        self.heading_std = heading_std
        self.acceleration_std = acceleration_std
        self.angular_acceleration_std = angular_acceleration_std
        self.angular_velocity_std = angular_velocity_std

        self.acceleration_function = acceleration_function
        self.angular_acceleration_function = angular_acceleration_function
        self.angular_velocity_function = angular_velocity_function
        self.heading_function = heading_function


    def get_data(self) -> dict:
        rotation = (self.heading_function() + std(self.heading_std)).item()
        rot = R.from_euler('z', rotation)
        global_accel = self.acceleration_function() + std(self.acceleration_std, 3)
        
        acceleration = rot.apply(global_accel)
        
        angular_acceleration = self.angular_acceleration_function() + std(self.angular_acceleration_std)
        gyro = (self.angular_velocity_function() + std(self.angular_velocity_std)).item()
        return {"rotation": rot, "acceleration": acceleration, "angular_acceleration": angular_acceleration, "gyro": gyro, "heading": rotation}

    def initialize(self):
        pass

    def overview(self) -> str:
        return f"Simulated IMU"
    
class FakeGPS(Sensor):
    """
    A simulated GPS to provide data in the correct format from the simulation.
    """

    def __init__(self, position_std: float, position_function: callable):
        self.position_function = position_function
        self.position_std = position_std

    def get_position(self):
        """
        Gets simulation position data.
        """
        return self.position_function() + std(self.position_std, 2)
    def get_data(self) -> dict:
        return {"position": self.get_position()}

    def initialize(self):
        pass

    def overview(self) -> str:
        return f"Simulated GPS"
    
class FakeDVL(Sensor):
    """
    A simulated DVL to provide data in the correct format from the simulation.
    """

    def __init__(self, velocity_std: float, velocity_function: callable):
        self.velocity_std = velocity_std
        self.velocity_function = velocity_function

    def get_velocity(self):
        """
        Gets simulation velocity data.
        """
        return self.velocity_function() + std(self.velocity_std, 3)

    def get_data(self) -> dict:
        return {"velocity": self.get_velocity()}

    def initialize(self):
        pass

    def overview(self) -> str:
        return f"Simulated DVL"
    
class FakeCamera(Sensor):
    """
    A simulated Camera to provide data in the correct format from the simulation.
    """

    def __init__(self, obstacle_std: float, obstacle_function: callable):
        self.obstacle_std = obstacle_std
        self.obstacle_function = obstacle_function

    def get_obstacles(self):
        """
        Gets simulation obstacle data.
        """
        return self.obstacle_function(self.obstacle_std)

    def get_data(self) -> dict:
        return {"obstacles": self.get_obstacles()}

    def initialize(self):
        pass

    def overview(self) -> str:
        return f"Simulated Camera"