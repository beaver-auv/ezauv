#Code is broken. Will be scrapped later

import numpy as np

from ezauv.auv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils.inertia import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector
from ezauv.mission.tasks.subtasks import HeadingPID, Simulate
from ezauv.mission import Path
from ezauv.simulation.core import Simulation


motor_locations = [
    np.array([-1., -1., 0.]),
    np.array([-1., 1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.])
]

motor_directions = [
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.])
] # this debug motor configuration is the same as bvr auv's hovercraft



bounds = [[-1, 1]] * 4 # motors can't go outside of (-100%, 100%)...
deadzone = [[-0.1, 0.1]] * 4 # or inside (-10%, 10%), unless they equal 0 exactly

sim = Simulation(motor_locations, motor_directions, 1/6, bounds, deadzone)

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
                ]
        ),
        sensors = SensorInterface(imu=sim.imu(0.05), depth=sim.depth(0.)),
    )

sim_anchovy.register_subtask(Simulate(sim)) # gotta make sure it knows to simulate the sub
sim_anchovy.register_subtask(HeadingPID(0, 0.03, 0.0, 0.01)) # this will keep it facing straight


mission = Path(
    AccelerateVector(np.array([1., 0., 0., 0., 0., 0.]), 2), # start by going forward
    AccelerateVector(np.array([-1, 0., 0., 0., 0., 0.]), 2), # slow down...
    AccelerateVector(np.array([0., 0., 0., 0., 0., 100.]), 10)) # spin as fast as you can!

sim_anchovy.travel_path(mission)

sim.render()
