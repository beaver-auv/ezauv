import math as m
import pybullet as p
import pybullet_data
import numpy as np
from time import sleep

# Later versions of the sim will define motor as a class
class SimMotor:
    # Every motor had attributes
    def __init__(self, angle, min_rpm, max_rpm, pos_x, pos_y, is_up_motor):
        self.angle = angle
        self.min_rpm = min_rpm
        self.max_rpm = max_rpm
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.is_up_motor = is_up_motor
    # Not finished yet however, in the future this will calculate thrust
    def start_motor(self, rpm):
        #this will be added later
        #we need to calculate a thrust force from rpm
        #for now this program calculates x and y thrust forces using the rpm as a base
        if self.min_rpm > rpm or rpm < self.max_rpm:
            raise Exception("The rpm specified was not with range of the of the motor limits")
        elif self.is_up_motor == True:
            return[0, 0, rpm]
        else:
            return [(rpm * m.cos(self.angle)), (rpm * m.tan(self.angle)), 0]