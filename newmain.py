from ezauv.simulation import SimAUV, SimMotor, SimWorld
import pybullet as p

Sub = SimAUV(10, 0.05, 0.1, [])

World = SimWorld(1000, -9.81, 0.9, Sub)

World.init()

pos, orn = p.getBasePositionAndOrientation(Sub.sub)
x, y, z = pos

#Side Motors
Sub.motor_add(SimMotor(-45, 0, 100, x + 0.1, y + 0.1, False))
Sub.motor_add(SimMotor(45, 0, 100, x - 0.1, y + 0.1, False))
Sub.motor_add(SimMotor(45, 0, 100, x - 0.1, y - 0.1, False))
Sub.motor_add(SimMotor(-45, 0, 100, x + 0.1, y - 0.1, False))

#Top Motors
Sub.motor_add(SimMotor(0, 0, 100, x + 0.1, y + 0.1, True))
Sub.motor_add(SimMotor(0, 0, 100, x - 0.1, y + 0.1, True))
Sub.motor_add(SimMotor(0, 0, 100, x - 0.1, y - 0.1, True))
Sub.motor_add(SimMotor(0, 0, 100, x + 0.1, y - 0.1, True))
