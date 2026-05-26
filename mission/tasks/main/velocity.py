from ezauv.mission.mission import Task
from ezauv import AccelerationState, TotalAccelerationState, VelocityState
from ezauv.utils import PID
from typing import Union
import numpy as np
import time
from abc import ABC, abstractmethod
from itertools import zip_longest

class VelocityTask(Task, ABC):

    def __init__(self, Kp, Ki, Kd, RKp, RKi, RKd):
        super().__init__()
        self.linear_pids = [PID(Kp,Ki,Kd, 0) for _ in range(3)]
        self.rotational_pids = [PID(RKp, RKi, RKd, 0) for _ in range(3)]
        
    def update(self) -> np.ndarray:
        signals = []

        target = self.velocity()

        local_linear_velocities = self.map.global_to_local_vector(self.map.full_velocities[0:3])
        local_rotational_velocities = self.map.global_to_local_vector(self.map.full_velocities[3:6])
        local_velocities = np.concatenate((local_linear_velocities, local_rotational_velocities))
        for pid, measurement, wanted in zip_longest(self.linear_pids + self.rotational_pids, local_velocities, np.concatenate((target.translation, target.rotation))):
            if wanted is not None:
                signals.append(pid.signal(measurement - wanted))
            else:
                signals.append(0.)

        acceleration_state = AccelerationState(local=True)
        acceleration_state.translation = signals[:3]
        acceleration_state.rotation = signals[3:]
        # print("Position:", np.round(self.map.position, 2))
        # print("Velocity measurements:", np.round(local_velocities, 2))
        # print("Acceleration signals:", np.round(acceleration_state.translation, 2), np.round(acceleration_state.rotation, 2))
        # print("Target velocity:", [np.round(t, 2) for t in target.translation if t is not None], [np.round(r, 2) for r in target.rotation if r is not None])
        # print("\n\n\n")
       
        # print("Velocity errors:", np.round([m - w for m, w in zip(self.map.full_velocities, np.concatenate((target.translation, target.rotation))) if w is not None], 2))
        return acceleration_state
        
    @abstractmethod
    def velocity(self) -> VelocityState:
        """Returns a VelocityState the robot will try to follow."""
        pass