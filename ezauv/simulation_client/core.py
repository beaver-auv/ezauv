import socket
import math
import json
from ezauv.simulation.fake_sensors import FakeIMU
from ezauv.simulation.fake_clock import FakeClock
from ezauv.mission import Subtask
from scipy.spatial.transform import Rotation as R

class SimulationClient:
    def __init__(self, host="127.0.0.1", port=8080):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setblocking(False)
        self.fake_clock = FakeClock()
        self.last_imu_data = {"rotation":0.0, "acceleration":[0.0,0.0]}
    def connect(self):
        self.socket.connect((self.host, self.port))
    def encode_motor_data(index, speed):
        return f"{str(index)};{str(speed)}"
    def set(self, index, speed):
        self.socket.sendall(self.encode_motor_data(index, speed))
    def set_motor(self, index):
        return lambda speed: self.set(index, speed)
    def fetch_data(self):
        data = self.socket.recv(1024)
        if data and len(data)>0:
            self.last_imu_data = json.decode(data)
    def clock(self):
        return self.fake_clock
    def imu(self, dev):
        return FakeIMU(dev, lambda:self.last_imu_data["acceleration"], lambda:R.from_euler('z', self.last_imu_data["rotation"], degrees=True))

class UpdateSimClient(Subtask):
    def __init__(self, client:SimulationClient):
        self.client = client
    def name(self) -> str:
        return "UpdateSimClient"
    def update(self, sensors: dict):
        self.client.fetch_data()