import numpy as np

class Map:
    """
    A class which builds a map of the environment based on sensor data and is fed into Task objects.
    The baseline Map class has little functionality, so it should be extended upon if information 
    about the environment is known.
    """
    def __init__(self):
        self.sensor_data = {}
    
    def update(self, sensor_data):
        self.sensor_data = sensor_data

    def kill(self):
        pass