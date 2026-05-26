from ezauv.simulation.core import Simulation
from ezauv.map.roboboat_map import ColorObstacle, Color
import numpy as np

class RoboBoatCore(Simulation):
    def __init__(self, motor_locations, motor_directions, bounds, deadzone, seed=10, coefficients=[0, 1], random_map=False):
        super().__init__(motor_locations, motor_directions, bounds, deadzone, seed=seed, coefficients=coefficients)
        self.random_map = random_map
        # end_location = self.place_gate_challenge(np.array([10.0, 0]), np.array([1.0, 0]))
        self.place_gate_challenge( np.array([-20.0, -45.0]), np.array([1.0, 0]))
        # self.place_navigation_challenge( np.array([-5.0, -43.0]), np.array([1.0, 0]))
        # self.place_speed_challenge(np.array([0.0, 0.0]), np.array([-1.0, 0.0]))
        self.location = np.array([-45.0, -45.0])

    def range(self, min, max):
        if self.random_map:
            return np.random.uniform(min, max)
        else:
            return (min + max) / 2
        
    def place_obstacle(self, position: np.ndarray, color: Color = Color.BLACK):
        obstacle = ColorObstacle(position, 5/12, color)
        self.obstacles.append(obstacle)

    def place_beacon(self, position: np.ndarray, green: bool):
        color = Color.GREEN if green else Color.RED
        obstacle = ColorObstacle(position, 1.5, color, beacon=True)
        self.obstacles.append(obstacle)

    def place_gate(self, position: np.ndarray, facing: np.ndarray, width: float = None):
        if width is None:
            width = self.range(6.0, 10.0)
        
        perpindicular = np.array([-facing[1], facing[0]])/np.linalg.norm(facing)
        self.place_obstacle(position + perpindicular * -width/2, Color.RED)
        self.place_obstacle(position + perpindicular * width/2, Color.GREEN)

    def place_gate_challenge(self, position: np.ndarray, facing: np.ndarray):
        # gate_distance = self.range(25.0, 100.0)
        gate_distance = 10.0
        self.place_gate(position, facing)
        self.place_gate(position + (facing / np.linalg.norm(facing)) * gate_distance, facing)
        return position + (facing / np.linalg.norm(facing)) * gate_distance

    def place_navigation_challenge(self, start_position: np.ndarray, start_facing: np.ndarray):
        theta = np.arctan2(start_facing[1], start_facing[0])
        num_gates = int(self.range(3, 7))
        gate_spacing = self.range(10.0, 25.0)
        theta_variation = self.range(np.pi/12, np.pi/4)

        position = start_position.copy()
        facing = start_facing.copy()
        self.place_gate(position, facing)
        for _ in range(num_gates - 1):
            position += (facing / np.linalg.norm(facing)) * gate_spacing
            theta += theta_variation
            facing = np.array([np.cos(theta), np.sin(theta)])
            self.place_gate(position, facing)

        num_beacons = 2
        num_obstacles = 4
        zone_size = 24.0
        
        C = []
        buffer = 1
        for r in ([1.5 + buffer]*num_beacons + [5/12 + buffer]*num_obstacles):
            for _ in range(50_000):
                p = np.random.rand(2) * (zone_size - 2*r) + r
                if all(np.linalg.norm(p-q) >= r+s for q,s in C):
                    C.append((p, r))
                    break
            else:
                continue  # could not place all objects, skip

        R = np.array([[np.cos(theta - np.pi/2), -np.sin(theta - np.pi/2)],
                      [np.sin(theta - np.pi/2),  np.cos(theta - np.pi/2)]])
        
        zone_center = position + (facing / np.linalg.norm(facing)) * (zone_size / 2)
        # add debug objects at corners
        # C.extend([ (np.array([0, 0]), 5/12 + buffer),
        #            (np.array([zone_size, 0]), 5/12 + buffer),
        #            (np.array([0, zone_size]), 5/12 + buffer),
        #            (np.array([zone_size, zone_size]), 5/12 + buffer)
        #          ])
        for i, (local_pos, radius) in enumerate(C):
            world_pos = R @ (local_pos - np.array([zone_size/2, zone_size/2])) + zone_center
            if radius == 1.5 + buffer:
                self.place_beacon(world_pos, green=bool(i % 2))
            else:
                self.place_obstacle(world_pos)


    def place_speed_challenge(self, start_position: np.ndarray, start_facing: np.ndarray):
        gate_size = self.range(6.0, 10.0)
        buoy_distance = 20.0
        buoy_offset = self.range(-10, 10)
        beacon_offset = self.range(-5, 5)
        beacon_distance = buoy_distance / 2

        self.place_gate(start_position, start_facing, gate_size)
        self.place_obstacle(
            start_position + (start_facing / np.linalg.norm(start_facing)) * buoy_distance + (np.array([-start_facing[1], start_facing[0]])/np.linalg.norm(start_facing)) * -buoy_offset,
            Color.YELLOW
        )
        self.place_beacon(
            start_position + (start_facing / np.linalg.norm(start_facing)) * beacon_distance + (np.array([-start_facing[1], start_facing[0]])/np.linalg.norm(start_facing)) * beacon_offset,
            green=np.random.choice([True, False])
        )