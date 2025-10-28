import numpy as np

from ezauv.auv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils.inertia import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector
from ezauv.mission.tasks.subtasks import HeadingPID, Simulate
from ezauv.mission import Path
from ezauv.simulation.core import Simulation
from ezauv import AccelerationState, TotalAccelerationState

motor_locations = [
    np.array([-1., 1., 0.]),    # motor 2
    np.array([-1., -1., 0.]),   # motor 3
    np.array([1., 1., 0.]),     # motor 4
    np.array([1., -1., 0.]),    # motor 5
]

motor_directions = [
    np.array([1., 1., 0.]),     # motor 2
    np.array([1., -1., 0.]),    # motor 3
    np.array([1., -1., 0.]),    # motor 4
    np.array([1., 1., 0.]),     # motor 5
]   # this debug motor configuration is the same as bvr auv's hovercraft


bounds = [[-0.9, 0.9]] * 4 # motors can't go outside of (-90%, 90%)...
deadzone = [[-0.1, 0.1]] * 4 # or inside (-10%, 10%), unless they equal 0 exactly

degrees = [
    -0.01,
    0.4,
    -0.4,
    1.4
]   # this defines our motor's pwm -> thrust curve as t = -0.01 + 0.4p - 0.4p^2 + 1.4p^3


sim = Simulation(motor_locations, motor_directions, bounds, deadzone, degrees)

sim_anchovy = AUV(
    motor_controller = MotorController(
        inertia = InertiaBuilder(
            Cuboid(
                mass=1,
                width=1,
                height=1,
                depth=0.1,
                center=np.array([0,0,0])
            )).moment_of_inertia(), # the moment of inertia helps with rotation

            motors = [
                Motor(
                    direction,
                    loc,
                    sim.set_motor(i),
                    lambda: 0,
                    Motor.Range(bounds[i][0], bounds[i][1]),
                    Motor.Range(deadzone[i][0], deadzone[i][1])
                    )
                for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))
                ],
            coefficients = degrees
        ),
        sensors = SensorInterface(sensors=[sim.imu(0.05)]),
        lock_to_yaw = False,
        clock = sim.clock(),
    )

sim_anchovy.register_subtask(Simulate(sim)) # gotta make sure it knows to simulate the sub

mission = Path(
    AccelerateVector(AccelerationState(Tx=1, local=False), 3),      # start by going right locally,
    AccelerateVector(AccelerationState(Tx=-1, local=False), 3),     # then slow down by going left locally,
    AccelerateVector(AccelerationState(Rz=-20, local=False), 5),    # then spin really fast,
    AccelerateVector(AccelerationState(Tx=-2, local=False), 5),    # then go left globally, while spinning
)

sim_anchovy.travel_path(mission)

sim.render() # this draws an animation using pygame; you can see it in videos/animation.mp4