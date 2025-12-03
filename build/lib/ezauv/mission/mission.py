from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np
from ezauv import TotalAccelerationState
from ezauv.utils import Clock

class Task(ABC):

    @abstractmethod
    def name(self) -> str:
        """The name of the task."""
        pass

    @abstractmethod
    def finished(self) -> bool:
        """Whether the task has completed."""
        pass

    @abstractmethod
    def update(self, sensor_data: dict) -> TotalAccelerationState:
        """Update based on sensor data."""
        pass

    def __init__(self, clock=Clock()):
        self.clock = clock

class Subtask(ABC):
    @abstractmethod
    def name(self) -> str:
        """The name of the subtask."""
        pass

    @abstractmethod
    def update(self, sensor_data: dict) -> TotalAccelerationState:
        """Update direction based on sensor data. Does not directly set the direction, only adds to it."""
        pass

    def __init__(self, clock=Clock()):
        self.clock = clock


class Path:
    """
    Defines a list of `Task`s to be executed in succession.
    """
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args
