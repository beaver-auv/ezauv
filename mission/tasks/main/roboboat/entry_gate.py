from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.waypoint import WaypointTask
from ezauv.map import CircleGridObject
from ezauv import AccelerationState, TotalAccelerationState
from ezauv.communications.report_pb2 import *
from typing import Union
import numpy as np
import time

class EntryGate(WaypointTask):
    def __init__(self, TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, lookahead_distance=0.5, slowing_distance=5.0, stopping_distance=1.0, allow_backup=True, allow_sideways=True, reverse=False):

        self.first_gate = None
        self.second_gate = None

        self.traveled_first_gate = False
        self.traveled_second_gate = False
        self.reverse = reverse
        self.starting_heading = None
        
        super().__init__(TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, lookahead_distance=lookahead_distance, slowing_distance=slowing_distance, stopping_distance=stopping_distance, allow_backup=allow_backup, allow_sideways=allow_sideways)

    def start(self, map):
        super().start(map)
        map.current_task = TaskType.TASK_ENTRY_EXIT

    def finished(self) -> bool:
        was_traveled_first_gate = self.traveled_first_gate

        if self.first_gate and not self.traveled_first_gate:
            self.traveled_first_gate = self.map.stopped_at(self.first_gate.center, radius=self.stopping_distance)
        if self.second_gate and not self.traveled_second_gate:
            self.traveled_second_gate = self.map.stopped_at(self.second_gate.center, radius=self.stopping_distance)

        if self.traveled_first_gate and not was_traveled_first_gate:
            self.fresh_waypoint = True
        return self.traveled_first_gate and self.traveled_second_gate
    
    def waypoint(self) -> np.ndarray:
        if self.starting_heading is None:
            self.starting_heading = self.map.heading

        if(self.map.entry_gates):
            self.first_gate, self.second_gate = self.map.entry_gates
            if self.reverse:
                self.first_gate, self.second_gate = self.second_gate, self.first_gate
        elif not self.first_gate or not self.second_gate:
            gates = self.map.identify_gates()
            
            # sort gates by distance
            gates = sorted(gates, key=lambda g: self.map.distance(g.center))
            if self.first_gate is None and len(gates) >= 1:
                self.first_gate = gates[0]
            if self.second_gate is None and len(gates) >= 2:
                self.second_gate = gates[1]
        else:
            self.map.entry_gates = [self.first_gate, self.second_gate]

        obstacles = []
        if(self.first_gate and self.second_gate):
            obstacles = self.first_gate.generate_obstacles(self.second_gate)

        if self.first_gate and not self.traveled_first_gate:
            return [self.first_gate.generate_goal(), obstacles]
        elif self.second_gate and not self.traveled_second_gate:
            return [self.second_gate.generate_goal(), obstacles]
        elif not self.traveled_first_gate or not self.traveled_second_gate:
            # if we haven't identified enough gates, just go forward
            return CircleGridObject(self.map.position + 10.0 * np.array([np.cos(self.starting_heading), np.sin(self.starting_heading)]), 1.0)
            # raise Exception("Not enough gates identified on the map.")
        else:
            # both gates traveled, we're finished
            return CircleGridObject(self.map.position, 1.0)

    def name(self) -> str:
        return "Travel entry gate"