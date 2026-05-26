from ezauv.mission.mission import Subtask
from ezauv.utils.pid import PID
from ezauv import TotalAccelerationState, AccelerationState

import numpy as np


class HeadingPID(Subtask):

    def __init__(self, wanted_heading, Kp, Ki, Kd):
        super().__init__()
        self.pid = PID(Kp, Ki, Kd, 0)
        self.wanted = wanted_heading

    def name(self) -> str:
        return "Heading PID"

    def update(self, sensor_data: dict) -> np.ndarray:
        q = sensor_data["rotation"]
        if q is None:
            raise Exception("Heading PID cannot run without rotation data")
        
        
        yaw = q.as_euler('zyx', degrees=True)[0]
        diff = self.wanted - yaw
        
        if abs(diff) >= 180:
            sign = yaw / abs(yaw)
            abs_diff_yaw = 180 - abs(yaw)
            abs_diff_target = 180 - abs(self.wanted)
            diff = sign * (abs_diff_yaw + abs_diff_target)

        signal = self.pid.signal(-diff)
        return AccelerationState(Rz=signal, local=True)