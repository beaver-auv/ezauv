from ezauv.map.flat_map import FlatMap
# from ezauv.map.grid import Grid
from ezauv.map.grid_objects import LineGridObject
from ezauv.map.obstacle_map import Obstacle, ObstacleMap
from ezauv.map.path import Path
from enum import Enum
from time import time
import numpy as np
from ezauv.simulation.animator import set_obstacles
from typing import Self
from ezauv.communications.report_pb2 import *
from ezauv.telemetry import TELEMETRY


class Color(Enum):
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"
    BLACK = "black"

class ColorObstacle(Obstacle):
    def __init__(self, position, radius, color: Color, beacon=False, lifetime=np.inf):
        super().__init__(position, radius, lifetime)
        self.color = color
        self.beacon = beacon

class Gate:
    def __init__(self, center: np.ndarray, facing: np.ndarray, width: float):
        self.center = center
        self.facing = facing / np.linalg.norm(facing)
        self.width = width

    def obstacles(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Return the positions of the left and right obstacles that make up the gate."""
        perpindicular = np.array([-self.facing[1], self.facing[0]])
        left_obstacle = self.center + perpindicular * -self.width/2
        right_obstacle = self.center + perpindicular * self.width/2
        
        return (left_obstacle, right_obstacle)


    def generate_obstacles(self, other: Self) -> tuple[LineGridObject, LineGridObject]:
        """Generate a pair of pair of obstacles representing the path between two gates."""
        self_obstacles = self.obstacles()
        other_obstacles = other.obstacles()
        left_obstacle = LineGridObject(
            start=self_obstacles[0],
            end=other_obstacles[0],
            thickness=1
        )
        right_obstacle = LineGridObject(
            start=self_obstacles[1],
            end=other_obstacles[1],
            thickness=1
        )
        return (left_obstacle, right_obstacle)
    
    def generate_goal(self) -> LineGridObject:
        """Generate a line representing this gate's goal."""
        buoys = self.obstacles()
        return LineGridObject(
            start=buoys[0],
            end=buoys[1],
            thickness=1.0
        )
    

class RoboBoatMap(ObstacleMap):
    def __init__(self, 
                    max_velocity: float,
                    dimensions: tuple[tuple[float, float], tuple[float, float]],
                    bot_radius: float,
                    resolution: float,
                    
                    velocity_std: float,
                    position_std: float,
                    angle_std: float,
                    rotational_velocity_std: float
                 ):
        """
        Dimensions is a pair of tuples ((min_x, min_y), (max_x, max_y)). Keep in mind the bot starts at (0,0).
        Created based on the specifications of the 2026 RoboBoat competition.
        """
        R = np.diag([position_std**2, position_std**2, angle_std**2, velocity_std**2, velocity_std**2, rotational_velocity_std**2])
        super().__init__(max_velocity, dimensions, bot_radius, resolution, R=R)
        self.entry_gates = []
        self.navigation_gates = []
        self.speed_challenge_gate = None
        self.current_task = None

    def identify_gates(self):
        """Return a list of Gate objects identified in the map based on known red and green obstacles."""

        red_obstacles, green_obstacles = [], []
        for o in self.obstacles:
            if isinstance(o, ColorObstacle):
                if o.color == Color.RED:
                    red_obstacles.append(o)
                elif o.color == Color.GREEN:
                    green_obstacles.append(o)
        
        gates = []
        for red in red_obstacles:
            # a gate is defined by a red and green obstacle within some distance of each other
            # we can identify gates by making a circle around the red obstacle and seeing if any green obstacles are within that circle
            for green in green_obstacles:
                distance = np.linalg.norm(red.position - green.position)
                if distance < (10.0 + 1.0) and distance > (5.0 - 1.0):  # gate constraints, with one foot of tolerance
                    center = (red.position + green.position) / 2
                    perpindicular = (red.position - green.position) / distance
                    facing = np.array([-perpindicular[1], perpindicular[0]])
                    width = distance
                    gates.append(Gate(center, facing, width))
        return gates
    
    def identify_beacons(self):
        """Return a list of ColorObstacle objects identified as beacons in the map."""
        beacons = []
        for o in self.obstacles:
            if isinstance(o, ColorObstacle) and o.beacon:
                beacons.append(o)
        return beacons
    
    def identify_buoys(self, color):
        """Return a list of ColorObstacle objects identified as buoys in the map."""
        buoys = []
        for o in self.obstacles:
            if isinstance(o, ColorObstacle) and o.color == color and not o.beacon:
                buoys.append(o)
        return buoys

    def update(self, sensor_data):
        super().update(sensor_data)
        state = RobotState.STATE_AUTO
        position = LatLng()
        position.latitude = self.position[1]
        position.longitude = self.position[0]

        speed = np.linalg.norm(self.velocities)
        heading_deg = np.degrees(self.heading)
        current_task = self.current_task
        TELEMETRY.set_state(state, position, speed, heading_deg, current_task)