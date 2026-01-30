from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.waypoint import WaypointTask
from ezauv.map import CircleGridObject
from ezauv import AccelerationState, TotalAccelerationState
from ezauv.map.roboboat_map import Color
from typing import Union
import numpy as np
import time


class SpeedChallenge(WaypointTask):
    def __init__(self, TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, general_location, lookahead_distance=0.5, slowing_distance=5.0, stopping_distance=0.1, allow_backup=True, allow_sideways=True):

        self.gate = None
        self.clockwise = None
        self.buoy = None
        self.general_location = general_location
        self.passed_gate = False
        self.reached_buoy = False
        self.done = False

        self.target = None

        
        super().__init__(TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, lookahead_distance=lookahead_distance, slowing_distance=slowing_distance, stopping_distance=stopping_distance, allow_backup=allow_backup, allow_sideways=allow_sideways)

    def finished(self) -> bool:
        return self.done
    
    def waypoint(self) -> np.ndarray:
        
        if(self.map.speed_challenge_gate):
            self.gate = self.map.speed_challenge_gate
        elif not self.gate:
            gates = self.map.identify_gates()
            
            # sort gates by distance
            gates = sorted(gates, key=lambda g: self.map.distance(g.center))
            for gate in gates:
                if np.linalg.norm(gate.center - self.general_location) < 20.0:
                    if not any([np.allclose(gate.center, known_gate.center, atol=1) for known_gate in self.map.navigation_gates + self.map.entry_gates]):
                        self.gate = gate
                        break
        else:
            self.map.speed_challenge_gate = self.gate

        if not self.gate:
            # if we haven't identified the gate, just go to the general location
            return CircleGridObject(self.general_location, 3.0)

        elif self.gate and not self.passed_gate:
            if self.map.stopped_at(self.gate.center, radius=self.stopping_distance):
                self.passed_gate = True
            else:
                return self.gate.generate_goal()
        
        if self.clockwise is None:
            beacons = self.map.identify_beacons()
            if beacons:
                # pick the closest beacon
                beacon = sorted(beacons, key=lambda b: self.map.distance(b.position))[0]
                self.clockwise = beacon.color == Color.GREEN
        
        if not self.buoy:
            buoys = self.map.identify_buoys(Color.YELLOW)
            if buoys:
                # pick the closest buoy
                self.buoy = sorted(buoys, key=lambda b: self.map.distance(b.position))[0]

        if self.buoy and self.passed_gate:
            if not self.target:
                # find how far along the gate's facing axis the buoy is
                c = self.buoy.position - self.gate.center
                v = self.gate.facing
                n = np.dot(c, v) / np.dot(v, v)

                self.target = self.buoy.position + v * n * 1.25
            distance_to_target = self.map.distance(self.target)
            if distance_to_target < 3.0:
                if not self.reached_buoy:
                    self.reached_buoy = True
                    self.target = self.gate.center
                else:
                    self.done = True
            else:
                # calculate a point to circle the buoy
                direction_to_target = (self.target - self.map.position) / distance_to_target
                if self.clockwise:
                    tangent_direction = np.array([direction_to_target[1], -direction_to_target[0]])
                else:
                    tangent_direction = np.array([-direction_to_target[1], direction_to_target[0]])
                
                circle_point = self.target + direction_to_target * 5.0 + tangent_direction * 5.0
                return CircleGridObject(circle_point, 3.0)
            
        if self.done:
            return CircleGridObject(self.map.position, 1.0)  # stay in place
        

    def name(self) -> str:
        return "Speed challenge"