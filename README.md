# ezauv
**ezauv is a Python library to make AUVs, or autonomous underwater vehicles, easy.**
This library was created for BVR AUV, Beaver Country Day School's RoboSub team.

## Why?
In the yearly RoboSub competition, teams build autonomous submarines. Much of the code of these submarine can be abstracted into a few basic concepts: tasks, like travelling a vector or circling a buoy; subtasks, like holding heading or depth; and sensors, such as IMUs and depth sensors. ezauv provides an easy interface for all of these pieces of code to simplify their creation. More importantly, ezauv solves the motors needed to travel a specific vector and rotation under the hood, regardless of the thrust vectors or locations of the motors and the sub's overall dimensions. This allows the code for one hardware surface to be ported to another with almost no change to the code. ezauv also allows for arbitrary polynomial pwm-to-thrust motor curves.

ezauv also provides interfaces for logging data to file, building the inertia tensor of your sub from a set of geometries, and some simple tasks and subtasks such as accelerating in a direction or using a PID on rotation.

## Installation

 - Make sure pip is installed and working
 - Check your Python version; ezauv is built for Python 3.11, but will likely work on most versions >3
 - Run the following command with pip:
```
pip install ezauv
```

For developer information, [read the wiki](https://github.com/beaver-auv/ezauv/wiki).
## Example Program
This example creates a simulation with a square hovercraft, moves it forward, moves it backwards, and then spins it.
```python
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
```

## Important Note
- Currently, the library uses Gurobi. It installs a free license using gurobipy, but eventually it expires and either an academic or (very expensive) commerical license must be used. Eventually, this will be replaced with an open-source solver.
