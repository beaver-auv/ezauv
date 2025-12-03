from ezauv.mission.mission import Subtask
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.simulation.core import Simulation
from ezauv import AccelerationState

import numpy as np
import time

class Simulate(Subtask):

    def __init__(self, simulation: Simulation):
        super().__init__()
        self.simulation = simulation
        self.prevtime = -1.
        self.a=0

    def name(self) -> str:
        return "Simulate"

    def update(self, sensors: dict) -> np.ndarray:
        new_time = time.perf_counter()
        # print(f"{new_time=}, {self.prevtime=}")
        if(self.prevtime != -1):
            self.simulation.simulate(new_time - self.prevtime)
        self.prevtime = time.perf_counter()
        self.a=time.time()
        return AccelerationState()
        