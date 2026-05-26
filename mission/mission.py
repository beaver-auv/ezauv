from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np
from ezauv import TotalAccelerationState
from ezauv.utils import Clock
from ezauv.map.map import Map

class Task(ABC):
    def __init__(self):
        self.map: Map = None

    @abstractmethod
    def name(self) -> str:
        """The name of the task."""
        pass

    @abstractmethod
    def finished(self) -> bool:
        """Whether the task has completed."""
        pass

    @abstractmethod
    def update(self) -> TotalAccelerationState:
        """Update based on sensor data."""
        pass

    def replace(self) -> "Task":
        """Replace current task outputs with that of another task."""
        return None

    def start(self, map):
        self.map = map

    def wanted_acceleration(self, map) -> TotalAccelerationState:
        """Returns the last calculated wanted acceleration."""
        task = self.replace()
        if task is not None:
            if task.map is None:
                task.start(map)
            if not task.finished():
                return task.wanted_acceleration(map)
        if self.map is None:
            self.start(map)
        return self.update()

class Subtask(ABC):
    @abstractmethod
    def name(self) -> str:
        """The name of the subtask."""
        pass

    @abstractmethod
    def update(self) -> TotalAccelerationState:
        """Update direction based on sensor data. Does not directly set the direction, only adds to it."""
        pass

    def start(self, map):
        self.map = map


class Path:
    """
    Defines a list of `Task`s to be executed in succession.
    """
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args
