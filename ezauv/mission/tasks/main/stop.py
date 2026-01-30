from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.velocity import VelocityTask
from ezauv import AccelerationState, TotalAccelerationState
from typing import Union
import numpy as np
import math

from ezauv.velocity_state import VelocityState

class StopTask(VelocityTask):

    def __init__(self, TKp, TKi, TKd, RKp, RKi, RKd):
        super().__init__(TKp, TKi, TKd, RKp, RKi, RKd)
        self.path = None

    def start(self, map):
        super().start(map)
    
    def finished(self) -> bool:
        return self.map.is_stopped()
                
    def velocity(self) -> np.ndarray:
        return VelocityState(local=True, Rz=0.0, Tx=0.0, Ty=0.0)
    
    def name(self) -> str:
        return "Stop task"