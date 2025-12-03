from abc import ABC, abstractmethod

# The abstract interface for interacting with the hardware; this should be
# extended and registered into the auv


class Sensor(ABC):
    def __init__(self, log: callable):
        self.log = log

    @abstractmethod
    def initialize(self) -> None:
        """Initialize this sensor."""
        pass

    @abstractmethod
    def overview(self) -> None:
        """Log an overview of this sensor."""
        pass

    @abstractmethod
    def get_data(self) -> dict[str, object]:
        """
        Get the current data of this sensor, returned in the format of {name: data}, eg. {"depth": 15.4, "density": 0.6}.
        If multiple sensors return data under the same index, it is arbitrary which is returned.
        \n
        To provide rotation data, return a SciPy Rotation object under the key "rotation". Global
        rotations will automatically use this value.
        """
        pass


class SensorInterface:
    def __init__(self, sensors: list[Sensor]):
        self.sensors: list[Sensor] = sensors
        self.log = lambda str: print(str)

    def initialize(self) -> None:
        for sensor in self.sensors:
            sensor.log = self.log
            sensor.initialize()

    def overview(self) -> None:
        for sensor in self.sensors:
            sensor.overview()

    def get_data(self) -> dict[str, object]:
        data = {}
        for sensor in self.sensors:
            data.update(sensor.get_data())
        return data

