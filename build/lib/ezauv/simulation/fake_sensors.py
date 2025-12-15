from ezauv.hardware.sensor_interface import Sensor
import numpy as np

# a class to provide fake sensor data for the simulation
        
class FakeIMU(Sensor):
    """
    A simulated IMU to provide data in the correct format from the simulation.
    """

    def __init__(self, max_dev_accel: float, acceleration_function: callable, rotation_function: callable):
        self.max_dev_accel = max_dev_accel
        self.acceleration_function = acceleration_function
        self.rotation_function = rotation_function

    def get_accelerations(self):
        """
        Gets simulation acceleration data with error.
        """
        acceleration = self.acceleration_function()
        # acceleration = self.acceleration_function() + (np.random.rand(2) - 0.5) * 2 * self.max_dev_accel
        return np.append(acceleration, 0)  # bc it expects a 3d acceleration

    def get_data(self) -> dict:
        return {"rotation": self.rotation_function(), "acceleration": self.get_accelerations()}

    def initialize(self):
        pass

    def overview(self) -> str:
        return f"Simulated IMU -- Standard deviation: {self.max_dev_accel}"