from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.velocity import VelocityTask
from ezauv.utils.pid import PID
from ezauv import AccelerationState, TotalAccelerationState, VelocityState
from typing import Union
import numpy as np
import time
from abc import ABC, abstractmethod

class DirectVelocityTask(VelocityTask):

    def __init__(self, Kp, Ki, Kd, RKp, RKi, RKd, HKp, HKi, HKd):
        super().__init__(Kp, Ki, Kd, RKp, RKi, RKd)
        self.heading_pid = PID(HKp, HKi, HKd, 0)
        self.error = 0

        # self.max_heading_error = 0
        
    @abstractmethod
    def state(self):
        """Returns a tuple of (wanted heading, wanted forward velocity)."""
        pass

    def forward_vector(self, heading, forward):
        """Returns a global forward vector given a heading and wanted forward speed."""
        
        return np.array([forward * np.cos(heading),
                         forward * np.sin(heading),
                         0.0])

    def velocity(self):
        heading, forward = self.state()
        self.error = self.map.angle_to(heading)
        # if abs(self.error) > abs(self.max_heading_error):
            # self.max_heading_error = self.error
            # print("New max heading error (rad):", np.round(self.max_heading_error, 2))
            # print("Wanted heading (rad):", np.round(heading, 2), "Current heading (rad):", np.round(self.map.heading, 2))
        heading_signal = self.heading_pid.signal((self.error + np.pi) % (2 * np.pi) - np.pi)
        # print("Heading error (rad):", np.round(self.error, 2), "Heading signal:", np.round(heading_signal, 2))
        # print("Aiming for heading (rad):", np.round(heading, 2), "Current heading (rad):", np.round(self.map.heading, 2))
        forward_vector = self.forward_vector(heading, forward)

        # transform to local coordinates
        local_forward = self.map.global_to_local_vector(forward_vector)
        # print("Desired forward speed:", np.round(forward,2))
        # print("Forward vector (local):", np.round(local_forward,2))
        # print(heading_signal)
        return VelocityState(local=True, Rz=heading_signal, Tx=local_forward[0], Ty=local_forward[1])