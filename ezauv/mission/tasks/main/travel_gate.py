from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.waypoint import WaypointTask
from ezauv import AccelerationState, TotalAccelerationState
from typing import Union
import numpy as np
import time

class TravelGate(WaypointTask):
    def __init__(self, gate, TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, lookahead_distance=0.5, slowing_distance=5.0, stopping_distance=1.0, allow_backup=True, allow_sideways=True):
        self.gate = gate
        goal_position = gate.center + gate.facing * (gate.width / 2 + 1.0)  # 1 unit beyond the gate
        super().__init__(TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, goal=goal_position, lookahead_distance=lookahead_distance, slowing_distance=slowing_distance, stopping_distance=stopping_distance, allow_backup=allow_backup, allow_sideways=allow_sideways)

    def name(self) -> str:
        return "Travel gate"