import math as m
import pybullet as p
import pybullet_data
import numpy as np
from time import sleep

class SimAUV:
    def __init__(self, mass, volume, cross_sectional_area, motors):
        self.mass = mass
        self.volume = volume
        self.cross_sectional_area = cross_sectional_area
        self.motors = motors
        self.sub = p.loadURDF("cube_small.urdf", [0, 0, -1], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=2.0)
        if not isinstance(self.motors, list):
            raise Exception("Motors is not a list")
    def motor_add(self, motor):
        if isinstance(motor, list) == True:
            raise Exception("motor_add only accepts motors")
        else:
            self.motors.append(motor)
    def step(self):
        pos, orn = p.getBasePositionAndOrientation(self.sub)
        vel, ang_vel = p.getBaseVelocity(self.sub)
        if pos[2] < 0:
            buoyancy_force = WATER_DENSITY * self.volume * 9.81
            p.applyExternalForce(self.sub, -1, [0, 0, buoyancy_force], pos, p.WORLD_FRAME)
        drag_force = -DRAG_COEFFICIENT * np.array(vel) * np.linalg.norm(vel) * self.cross_sectional_area
        for i in len(self.motor):
            p.applyExternalForce(self.sub, -1, self.motor.start_motor(), [self.motor.pos_x, self.motor.pos_y, 0], p.WORLD_FRAME)