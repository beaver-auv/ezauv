import time
import numpy as np
import math
from ezauv.map.obstacle_map import Obstacle
from ezauv.simulation.animator import SimulationAnimator, set_obstacles
from ezauv.simulation.fake_sensors import FakeIMU, FakeGPS, FakeDVL, FakeCamera
from ezauv.simulation.fake_clock import FakeClock
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from ezauv.telemetry import TELEMETRY
# kinda sucks, 2d
class Simulation:

    def __init__(self, motor_locations, motor_directions, bounds, deadzone, seed=time.time(), coefficients=[0, 1], random_strength=0.):
        self.location = np.array([0., 0.])
        self.velocity = np.array([0., 0.])
        self.acceleration = np.array([0., 0.])
        self.rotation = 0 # radians
        # start by facing positive x axis
        self.rotational_velocity = 0
        self.rotational_acceleration = 0

        # normalize motor directions
        self.motor_locations = motor_locations # relative to center
        self.motor_directions = [direction/np.linalg.norm(direction) for direction in motor_directions]
        
        self.moment_of_inertia = 1/6

        self.timestep = 0.01 # in seconds
        self.prevtime = 0
        self.animator = SimulationAnimator(fps=1/self.timestep)

        self.drag = 0
        self.random_strength = random_strength

        self.bounds = bounds
        self.deadzone = deadzone

        self.rng = np.random.default_rng(int(seed))

        self.real_accel = np.array([0., 0.])
        self.real_angular_accel = 0
        self.real_velocity = np.array([0., 0.])
        self.real_angular_velocity = 0

        self.prev_vel = np.array([0., 0.])
        self.prev_rotational_velocity = 0
        self.prev_pos = np.array([0., 0.])
        self.prev_rot = 0

        self.fake_clock = FakeClock()
        self.time = 0
        self.temp_count = 0

        self.motor_matrix = np.array(self.motor_directions).T
        self.motor_magnitudes = np.zeros(len(motor_directions))

        self.polynomial = np.polynomial.Polynomial(coefficients)
        self.prev_time = -1

        self.obstacles = []
        self.last_wave = 0

    def get_random_push(self):
        linear_push = (self.rng.random(2) - 0.5) * 2 * self.random_strength
        rotational_push = (self.rng.random() - 0.5) * 2 * self.random_strength
        wave_chance = 0.5
        wave_strength = 100.0
        wave_cooldown = 1.0
        if np.random.random() < (wave_chance * self.timestep) and self.time - self.last_wave > wave_cooldown:
            wave_strength = self.random_strength * wave_strength
            linear_push *= wave_strength
            rotational_push *= wave_strength
            self.last_wave = self.time
        return linear_push, rotational_push

    
    def simulate(self, delta_time):
        # print(delta_time)
        delta_time += self.fake_clock.perf_counter() - self.time
        quantized_time = int(math.floor(self.time / self.timestep))
        quantized_new_time = int(math.floor((self.time + delta_time) / self.timestep))
        # quantizing before use prevents floating point errors
        # print("Total motor :", np.round(total_acceleration, 3), "m/s²")
        # print(self.motor_magnitudes)
        for timepoint in range(quantized_time, quantized_new_time):
            total_acceleration = np.zeros(3)
            motor_speeds = []
            for loc, direction, magnitude in zip(self.motor_locations, self.motor_directions, self.motor_magnitudes):
                motor_speeds.append(direction * magnitude)
                total_acceleration += direction * self.polynomial(magnitude)
            
            rotation = R.from_euler('z', self.rotation, degrees=False)
            rotated_locations = rotation.apply(self.motor_locations)
            rotated_speeds = rotation.apply(motor_speeds)

            random_linear, random_rotational = self.get_random_push()
            
            self.acceleration = rotation.apply(total_acceleration)[:2] # in global frame
            self.acceleration += random_linear
            # apply drag
            self.acceleration -= self.velocity * self.drag

            torque = 0
            for loc, direction, magnitude in zip(rotated_locations, self.motor_directions, self.motor_magnitudes):
                force = direction * self.polynomial(magnitude)
                force = rotation.apply(force)
                torque += np.cross(loc, force)[2]


            self.rotational_acceleration = torque / self.moment_of_inertia + random_rotational
            self.rotational_acceleration -= self.rotational_velocity * self.drag

            # self.real_accel = (self.velocity - self.prev_vel) / self.timestep
            # self.real_angular_accel = (self.rotational_velocity - self.prev_rotational_velocity) / self.timestep

            self.location += self.velocity * self.timestep
            self.velocity += self.acceleration * self.timestep

            self.rotational_velocity += self.rotational_acceleration * self.timestep
            self.rotation += self.rotational_velocity * self.timestep

            # visible_obstacles = self.points_in_camera(10, np.pi / 3)
            # set_obstacles(visible_obstacles)
            set_obstacles(self.obstacles)
            self.animator.append(
                self.location,
                self.rotation,
                self.velocity,
                [loc[:2] + self.location for loc in rotated_locations],
                [speed[:2] for speed in rotated_speeds]
            )
            # apply drag
        self.real_velocity = self.velocity.copy()
        self.real_angular_velocity = self.rotational_velocity.copy()
        # self.real_velocity = (self.location - self.prev_pos) / delta_time
        # self.real_angular_velocity = (self.rotation - self.prev_rot) / delta_time
        
        # self.real_accel = (self.real_velocity - self.prev_vel) / delta_time
        # self.real_angular_accel = (self.real_angular_velocity - self.prev_rotational_velocity) / delta_time
        # print("acc:", self.real_accel, self.acceleration)
        self.real_accel = self.acceleration.copy()
        self.real_angular_accel = self.rotational_acceleration.copy()
        # print("dt:", delta_time)

        self.time += delta_time
        self.fake_clock.set_time(self.time)
        # print(rot.apply(np.append(self.real_accel, [0.0])))
        # print("---Simulation Step---")
        # print("Total acceleration :", np.round(total_acceleration, 3), "m/s²")
        # print("Sim local acceleration:", np.round(local_accel, 3), "m/s²")
        # print("Motor magnitudes:", np.round(self.motor_magnitudes, 10))
        # print("\n\n\n")
        # print("Position:", np.round(self.location, 3))
        TELEMETRY.submit("real velocity x", self.real_velocity[0])
        TELEMETRY.submit("real velocity y", self.real_velocity[1])
        TELEMETRY.submit("real angular velocity", self.real_angular_velocity)
        
        
    def render(self):
        self.animator.render()

    def update_motor_speeds(self, speeds):
        for i, speed in enumerate(speeds):
            self.set(i, speed)
        
    
    def imu(self, heading_std: float, acceleration_std: float, angular_acceleration_std: float, angular_velocity_std: float):
        # rot = R.from_euler('z', self.rotation, degrees=False)
        # print(rot.apply(np.append(self.real_accel, [0.0])))
        return FakeIMU(heading_std, acceleration_std, angular_acceleration_std, angular_velocity_std,
                       lambda: np.append(self.real_accel, [0.0]), # global frame
                       lambda: self.real_angular_accel, 
                       lambda: self.real_angular_velocity,
                       lambda: self.rotation
        )

    def gps(self, position_std: float):
        return FakeGPS(position_std, lambda: self.location)
    
    def dvl(self, velocity_std: float = 0.01):
        return FakeDVL(velocity_std, lambda: {"translational": self.real_velocity, "rotational": np.array([0., 0., self.real_angular_velocity])})
    
    def points_in_camera(self, max_range, fov, std):
        visible = []

        for p in self.obstacles:
            v = p.position - self.location
            heading = self.rotation
            dist = np.linalg.norm(v)
            if dist > max_range:
                continue

            angle = np.arctan2(v[1], v[0])
            dtheta = np.arctan2(
                np.sin(angle - heading),
                np.cos(angle - heading)
            )

            if abs(dtheta) <= fov / 2:
                visible.append(p)
        # visible = self.obstacles
        visible = [deepcopy(o) for o in visible]

        for o in visible:
            o.position += self.rng.normal(0, std, 2)

        return visible

    def camera(self, obstacle_std: float = 0.05):

        return FakeCamera(obstacle_std, lambda std: self.points_in_camera(20, np.pi/2, std))
    
    def clock(self):
        return self.fake_clock

    def set(self, index, speed):
        clamped = max(min(speed, self.bounds[index][1]), self.bounds[index][0])
        if(self.deadzone[index][0] > clamped > self.deadzone[index][1]):
            clamped = 0
        self.motor_magnitudes[index] = clamped

    def motor_interface(self, motors_speeds):
        self.update_motor_speeds(motors_speeds)
        new_time = time.perf_counter()
        if(self.prev_time != -1):
            self.simulate(new_time - self.prev_time)
        self.prev_time = time.perf_counter()
    
    def set_motor(self, index):
        return lambda speed: self.set(index, speed)
    
    def apply_force(self, *, thrust, rotation):
        self.velocity += thrust
        self.rotational_velocity += rotation

    def place_obstacle(self, position, radius):
        self.obstacles.append(Obstacle(position, radius))
        # set_obstacles(self.obstacles)
