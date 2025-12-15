from ezauv.mission.mission import Task
from ezauv import AccelerationState, TotalAccelerationState
from typing import Union
import numpy as np
import time

class AccelerateVector(Task):

    def __init__(self, acceleration_state: Union[AccelerationState, TotalAccelerationState], length: float):
        super().__init__()
        self.acceleration_state = acceleration_state
        self.length = length
        self.start = -1

    def name(self) -> str:
        return "Accelerate at vector"
    
    def finished(self) -> bool:
        if(self.start == -1):
            self.start = self.clock.time()
        return (self.clock.time() - self.start) >= self.length

    def update(self, sensor_data: dict) -> np.ndarray:
        if(self.start == -1):
            self.start = self.clock.time()
        return self.acceleration_state
        