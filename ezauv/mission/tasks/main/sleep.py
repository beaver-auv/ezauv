from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.velocity import VelocityTask
from ezauv import AccelerationState, TotalAccelerationState
from typing import Union
import numpy as np
import math

from ezauv.velocity_state import VelocityState

class SleepTask(Task):

    def __init__(self, time):
        super().__init__()
        self.time_left = time
    
    def finished(self) -> bool:
        return self.time_left <= 0
    
    def update(self):
        self.time_left -= self.map.sensor_data['dt']
        return AccelerationState()
                
    def name(self) -> str:
        return "Sleep task"