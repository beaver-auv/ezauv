import numpy as np
import math
from ezauv.simulation.animator import SimulationAnimator
from ezauv.simulation.fake_sensors import FakeIMU
from ezauv.simulation.fake_clock import FakeClock
from scipy.spatial.transform import Rotation as R

# kinda sucks, 2d
class Simulation:

    def __init__(self, motor_locations, motor_directions, bounds, deadzone, coefficients=[0, 1]):
        self.location = np.array([0., 0.])
        self.velocity = np.array([0., 0.])
        self.acceleration = np.array([0., 0.])
        self.rotation = 0 # degrees
        # start by facing positive x axis
        self.rotational_velocity = 0
        self.rotational_acceleration = 0

        self.motor_locations = motor_locations # relative to center
        self.motor_directions = [direction/np.linalg.norm(direction) for direction in motor_directions]
        
        self.motor_speeds = np.array([np.array([0., 0., 0.]) for _ in motor_directions])
        self.moment_of_inertia = 1/6

        self.timestep = 0.01 # in seconds
        self.prevtime = 0
        self.animator = SimulationAnimator(fps=1/self.timestep)

        self.drag = 0

        self.bounds = bounds
        self.deadzone = deadzone

        self.rng = np.random.default_rng(int(100))

        self.real_accel = np.array([0., 0.])
        self.prev_vel = np.array([0., 0.])

        self.fake_clock = FakeClock()
        self.time = 0
        self.temp_count = 0

        self.polynomial = np.polynomial.Polynomial(coefficients)
    
    def simulate(self, delta_time):
        # print(delta_time)
        delta_time += self.fake_clock.perf_counter() - self.time
        quantized_time = int(math.floor(self.time / self.timestep))
        quantized_new_time = int(math.floor((self.time + delta_time) / self.timestep))
        # quantizing before use prevents floating point errors
        for timepoint in range(quantized_time, quantized_new_time):
            self.real_accel = (self.velocity - self.prev_vel) / self.timestep
            self.prev_vel = self.acceleration

            # self.velocity += (self.rng.random() - 0.5) * 0.01
            # self.rotational_velocity += ((2 * (self.rng.random() - 0.5))) * 0.1
            self.location += self.velocity * self.timestep
            self.velocity += self.acceleration * self.timestep
            self.rotation = (self.rotation + self.rotational_velocity * self.timestep) % 360

            self.rotational_velocity += self.rotational_acceleration * self.timestep
            rotation = R.from_euler('z', np.deg2rad(self.rotation))
            
            
            rotated_speeds = []
            rotated_locations = []
            torques = []
            for i in range(len(self.motor_speeds)):
                s = np.delete(rotation.apply(np.array([self.motor_speeds[i][0], self.motor_speeds[i][1], 0.0])), 2)
                l = np.delete(rotation.apply(np.array([self.motor_locations[i][0], self.motor_locations[i][1], 0.0])), 2)
                rotated_speeds.append(s)
                rotated_locations.append(l)
                torques.append(np.cross(l, s))
            self.acceleration = sum(rotated_speeds)
            torque = sum(torques)

            self.rotational_acceleration = torque / self.moment_of_inertia
            
            self.animator.append(
                self.location,
                self.rotation,
                self.velocity,
                [loc + self.location for loc in rotated_locations],
                rotated_speeds
            )
            self.rotational_velocity *= (1 - self.drag)
            self.velocity *= (1 - self.drag)
        self.time += delta_time
        self.fake_clock.set_time(self.time)

    def render(self):
        self.animator.render()

    def update_motor_speeds(self, speeds):
        for i, speed in enumerate(speeds):
            self.set(i, speed)
        
    
    def imu(self, dev):
        return FakeIMU(dev, lambda: self.real_accel, lambda: R.from_euler('z', self.rotation, degrees=True))

    def clock(self):
        return self.fake_clock

    def set(self, index, speed):
            
            speed = max(min(speed, self.bounds[index][1]), self.bounds[index][0])
            if(self.deadzone[index][0] > speed > self.deadzone[index][1]):
                speed = 0
            self.motor_speeds[index] = self.motor_directions[index] * self.polynomial(speed)
    
    def set_motor(self, index):
        return lambda speed: self.set(index, speed)
    
    def apply_force(self, *, thrust, rotation):
        self.velocity += thrust
        self.rotational_velocity += rotation
