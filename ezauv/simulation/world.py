import math as m
import pybullet as p
import pybullet_data
import time as t

class SimWorld:
    def __init__(self, WaterDensity, Gravity, DragCoefficient, AUVS):
        self.WaterDensity = WaterDensity
        self.Gravity = Gravity
        self.DragCoefficient = DragCoefficient
    def init(self):
        global WATER_DENSITY
        WATER_DENSITY = self.WaterDensity

        global GRAVITY
        GRAVITY = self.Gravity

        global DRAG_COEFFICIENT
        DRAG_COEFFICIENT = self.DragCoefficient

        global physicsClient
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, GRAVITY)

        p.loadURDF("plane.urdf")
    def ExecuteSimulation(self):
        while True:
            for i in range(0, len(self.AUVS)):
                AUV = self.AUVS[i]
                AUV.step()
            p.StepSimulation()
            t.sleep(1/240)
