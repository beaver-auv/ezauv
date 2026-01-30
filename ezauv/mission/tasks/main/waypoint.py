from ezauv.mission.mission import Task
from ezauv.mission.tasks.main.direct_velocity import DirectVelocityTask
from ezauv import AccelerationState, TotalAccelerationState
from ezauv.mission.tasks.main.stop import StopTask
from typing import Union
import numpy as np
import math
from ezauv.simulation.animator import set_waypoint, set_goal, set_text
import time


class WaypointTask(DirectVelocityTask):

    def __init__(self, TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd, goal = None, lookahead_distance=0.5, slowing_distance=5.0, stopping_distance=1.0, allow_backup=True, allow_sideways=True):
        self.goal = goal
        self.lookahead_distance = lookahead_distance
        self.slowing_distance = slowing_distance
        self.allow_sideways = allow_sideways
        self.stopping_distance = stopping_distance
        self.allow_backup = allow_backup
        self.final_heading = None
        super().__init__(TKp, TKi, TKd, RKp, RKi, RKd, HKp, HKi, HKd)
        self.path = None
        self.stop_task = StopTask(TKp, TKi, TKd, RKp, RKi, RKd)
        self.stopping = False
        self.fresh_waypoint = True
        self.wanted_ticket = None
        self.debug_empty_path_start_time = None
        self.debug_current_waypoint = None

    def start(self, map):
        super().start(map)
        # self.debug_current_waypoint = waypoint
        # set_goal(np.array([self.path.end[0], self.path.end[1]]))
        # self.final_heading = self.get_final_heading(self.path)
    
    def get_final_heading(self, path):
        if(len(path.waypoints) < 2):
            return self.map.heading
        return math.atan2(path.end[1] - path.waypoints[-2][1],
                          path.end[0] - path.waypoints[-2][0])
    
    def finished(self) -> bool:
        if self.path is None:
            return False
        return self.map.stopped_at(self.path.end, radius=self.stopping_distance)
            
    def state(self) -> np.ndarray:
        if self.wanted_ticket is None and self.fresh_waypoint or (self.path is not None and self.map.need_replan(self.path)):
            self.fresh_waypoint = False
            obstacles = []
            goal = self.goal
            if(type(self.goal) == list):
                obstacles = self.goal[1]
                goal = self.goal[0]
            self.wanted_ticket = self.map.generate_path(goal, temporary_obstacles=obstacles)
            self.debug_current_waypoint = goal

        if self.wanted_ticket is not None:
            new_path = self.map.get_path(self.wanted_ticket)
            if new_path is not None:
                # print(new_path.waypoints)
                self.path = new_path
                self.wanted_ticket = None
            
                # set_text("")
                self.debug_empty_path_start_time = None
                self.final_heading = self.get_final_heading(self.path)
                set_goal(np.array([self.path.end[0], self.path.end[1]]))
        if self.wanted_ticket is not None:
            if self.debug_empty_path_start_time is None:
                self.debug_empty_path_start_time = time.time()
            # set_text(str(np.round(time.time() - self.debug_empty_path_start_time, decimals=3)))
            if self.path is None:
                return (self.map.heading, 0.0)
        set_waypoint(self.path.waypoints)

        
        
            # print("Replanned path to", self.path.end)
            # print(w.x, w.y)
            # print(self.path.waypoints)
            # print("Current position:", self.map.position)
        
        target = self.map.waypoint_at_distance(self.path, self.lookahead_distance)
        angle = self.map.angle_to_position(target)
        # print("Target waypoint:", np.round(target, 2))
        # get the forward speed
        d = self.map.distance(self.path.end)
        speed_scale = min(1., d / self.slowing_distance)
        heading_alignment = np.cos(self.map.angle_to(angle))
        if self.allow_sideways or not self.allow_backup:
            heading_alignment = max(0., heading_alignment)
        forward = (self.map.max_speed 
                   * heading_alignment
                   * speed_scale
                   )
        # print("Distance to goal:", np.round(d,2), "Speed scale:", np.round(speed_scale,2), "Heading alignment:", np.round(heading_alignment,2))
        if self.allow_backup and not self.allow_sideways:
            if abs(self.map.angle_to(angle)) > np.pi / 2:
                angle = (angle + np.pi) % (2 * np.pi)

        # if we're at the goal, set heading to final heading and stop
        # set_text(f"Distance to goal: {np.round(d,2)}")
        if d < self.stopping_distance:
            angle = self.final_heading
            forward = 0.0
            if not self.stopping and not (self.fresh_waypoint or self.wanted_ticket):
                self.stopping = True
        # print("Forward speed:", np.round(forward,2))
        # print(angle)
        return (angle, forward)
    
    def forward_vector(self, heading, forward):
        # print("Heading:", np.round(heading,2), "Forward:", np.round(forward,2))
        if not self.allow_sideways or self.path is None:
            return super().forward_vector(heading, forward)
        else:
            heading_to_target = self.map.angle_to_position(self.map.waypoint_at_distance(self.path, self.lookahead_distance))
            # print("Heading to target:", np.round(heading_to_target,2))
            vector = np.array([np.cos(heading_to_target), np.sin(heading_to_target), 0.0])
            # print("Raw vector:", np.round(vector,2))
            unit_vector = vector / np.linalg.norm(vector)
            return unit_vector * forward
        
    def waypoint(self):
        """Returns a WaypointObject which the sub will pathfind to."""
        return self.goal
    

    
    def name(self) -> str:
        return "Waypoint task"
    
    def replace(self):
        waypoint = self.waypoint()

        if (waypoint != self.goal):
            self.fresh_waypoint = True
            self.stopping = False
            self.goal = waypoint
        
        if self.stopping and self.stop_task is not None and not self.fresh_waypoint:
            return self.stop_task
        return None