from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.waypoint import WaypointTask
from ezauv.map.grid_objects import CircleGridObject, LineGridObject
from ezauv import AccelerationState, TotalAccelerationState
from typing import Union
import numpy as np
import time
from enum import Enum
from ezauv.map.roboboat_map import Color

class Stage(Enum):
    TRAVELING_GATES = 1
    NAVIGATING_ZONE = 2
    RETURNING_GATES = 3


class NavigationChannel(WaypointTask):
    def __init__(self, number_gates, number_green_beacons, rough_zone_size, TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, lookahead_distance=0.5, slowing_distance=5.0, stopping_distance=1.0, allow_backup=True, allow_sideways=True):

        self.number_gates = number_gates
        self.traveled = [False] * number_gates
        self.current_gate_index = 0
        self.wanted_gate_index = self.number_gates - 1
        self.current_gate = None
        self.current_facing = None
        self.known_gates = []

        self.stage = Stage.TRAVELING_GATES
        
        self.number_green_beacons = number_green_beacons
        self.number_beacons_circled = 0
        self.beacon_start_angle = None
        self.current_line_angle = None
        self.circled_beacons = []
        self.current_beacon = None
        self.rough_zone_size = rough_zone_size
        self.random_point = None

        self.returned = False

        super().__init__(TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, lookahead_distance=lookahead_distance, slowing_distance=slowing_distance, stopping_distance=stopping_distance, allow_backup=allow_backup, allow_sideways=allow_sideways)

    def finished(self) -> bool:
        return self.returned
    
    def waypoint(self) -> np.ndarray:
        if self.stage == Stage.TRAVELING_GATES:
            if self.map.navigation_gates:
                gates = self.map.navigation_gates
            elif not self.current_gate or self.current_gate_index >= len(self.known_gates):
                gates = self.map.identify_gates()
                
                # sort gates by distance
                gates = sorted(gates, key=lambda g: self.map.distance(g.center))

                # find the closest that's roughly in front of us and not already traveled
                if self.current_facing is None:
                    self.current_facing = np.array([np.cos(self.map.heading), np.sin(self.map.heading)])
                if self.current_gate is None:
                    for gate in gates:
                            similarity = np.dot(gate.facing, self.current_facing)
                            if similarity > 0 and not any([np.allclose(gate.center, known_gate.center, atol=1) for known_gate in self.known_gates + self.map.entry_gates]):
                                # 1 foot tolerance to consider same gate
                                self.current_gate = gate
                                self.current_facing = gate.facing
                                self.known_gates.append(gate)
                                break
                
            if self.current_gate is not None:
                goal = self.current_gate.generate_goal()
                if goal.sdf(self.map.position[0], self.map.position[1]) < 3.0:
                    self.traveled[self.current_gate_index] = True
                    if self.current_gate_index < self.wanted_gate_index:
                        self.current_gate_index += 1
                        self.fresh_waypoint = True
                    self.current_gate = None
                obstacles = []
                
                if self.current_gate_index > 0 and self.current_gate is not None:
                    obstacles = self.current_gate.generate_obstacles(self.known_gates[self.current_gate_index - 1])
                return [goal, obstacles]
            
            elif self.current_gate_index != self.wanted_gate_index:
                # just go forward until we find the next gate
                facing_vector = np.array([np.cos(self.map.heading), np.sin(self.map.heading)])
                forward_position = self.map.position + (facing_vector / np.linalg.norm(facing_vector)) * 5.0
                return CircleGridObject(forward_position, 0.1)
            else:
                # all gates traveled, we're finished with this stage
                self.map.navigation_gates = self.known_gates
                self.stage = Stage.NAVIGATING_ZONE
                return CircleGridObject(self.map.position, 1.0)  # stay in place

        elif self.stage == Stage.NAVIGATING_ZONE:
            beacons = self.map.identify_beacons()
            # sort beacons by distance
            if self.current_beacon is None:
                beacons = sorted(beacons, key=lambda b: self.map.distance(b.position))
                for beacon in beacons:
                    if beacon.color == Color.GREEN and not any([np.allclose(beacon.position, circled_beacon.position, atol=1) for circled_beacon in self.circled_beacons]):
                        self.current_beacon = beacon
                        break
            if self.current_beacon is not None:                
                # get the angle from the beacon
                angle_from_beacon = self.map.angle_from_position(self.current_beacon.position)
                
                if self.beacon_start_angle is None:
                    self.beacon_start_angle = angle_from_beacon
                    self.current_line_angle = 0
                # relativize angle to starting angle
                angle = (angle_from_beacon - self.beacon_start_angle) % (2 * np.pi)

                if(angle > self.current_line_angle) and (angle - self.current_line_angle < np.pi/4):
                    self.current_line_angle = angle
                
                line_length = 10.0
                line = LineGridObject(
                    self.current_beacon.position,
                    self.current_beacon.position + np.array([line_length * np.cos(self.current_line_angle + np.pi/8),
                                                            line_length * np.sin(self.current_line_angle + np.pi/8)]),
                    1.0
                )
                

                if abs(self.current_line_angle) >= 3*np.pi/2:
                    # completed a full (three-quarters, but close enough) circle
                    self.number_beacons_circled += 1
                    self.circled_beacons.append(self.current_beacon)
                    self.current_beacon = None
                    self.beacon_start_angle = None
                    self.random_point = None  # reset random point for next beacon
                    if self.number_beacons_circled >= self.number_green_beacons:
                        self.stage = Stage.RETURNING_GATES
                return line
            # choose a random point in the rough zone to navigate to until we find a beacon
            if self.random_point is None or self.map.distance(self.random_point) < 8.0:
                random_relative = np.random.uniform(-self.rough_zone_size/2, self.rough_zone_size/2, size=2)
                # rotate random point to be roughly in line with last gate facing

                rotation_matrix = np.array([[self.current_facing[0], -self.current_facing[1]],
                                            [self.current_facing[1], self.current_facing[0]]])
                self.random_point = (self.known_gates[-1].center + self.known_gates[-1].facing * (self.rough_zone_size / 2)) + rotation_matrix @ random_relative
            print("No beacon found, navigating to random point in zone:", self.random_point)
            return CircleGridObject(self.random_point, 5.0)
        elif self.stage == Stage.RETURNING_GATES:
            # return through the gates in reverse order
            if self.current_gate_index >= 0:
                if(self.current_gate is None):
                    self.current_gate = self.known_gates[self.current_gate_index]
                obstacles = []
                if self.current_gate_index < self.number_gates - 1:
                    obstacles = self.known_gates[self.current_gate_index + 1].generate_obstacles(self.current_gate)
                # print(obstacles)
                goal = self.current_gate.generate_goal()
                # print(goal.sdf(self.map.position[0], self.map.position[1]))
                if goal.sdf(self.map.position[0], self.map.position[1]) < 3.0:
                    self.current_gate_index -= 1
                    self.fresh_waypoint = True
                    self.current_gate = None
                return [goal, obstacles]
            else:
                # all gates traveled, we're finished
                self.returned = True
                return CircleGridObject(self.map.position, 1.0)  # stay in place

            

    def name(self) -> str:
        return "Travel navigation channel"