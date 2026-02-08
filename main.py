import numpy as np

from ezauv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector, WaypointTask
from ezauv.mission.tasks.main.roboboat.entry_gate import EntryGate
from ezauv.mission.tasks.subtasks import HeadingPID
from ezauv.mission import Path
from ezauv.simulation import Simulation
from ezauv.simulation.animator import set_waypoint
from ezauv import AccelerationState, TotalAccelerationState
from ezauv.map import Obstacle, ObstacleMap, CircleGridObject
from ezauv.simulation.roboboat_core import RoboBoatCore
from ezauv.map.roboboat_map import RoboBoatMap
from ezauv.mission.tasks.main.roboboat.navigation_channel import NavigationChannel
from ezauv.mission.tasks.main.roboboat.speed_challenge import SpeedChallenge
from ezauv.mission.tasks.main.sleep import SleepTask
from ezauv.telemetry import TELEMETRY


def main():
    motor_locations = [
        np.array([-1.5, 1., 0.]),  # motor 1
        np.array([-1.5, -1., 0.]),  # motor 2
        np.array([1.5, 1., 0.]),  # motor 3
        np.array([1.5, -1., 0.]),  # motor 4
    ]

    motor_directions = [
        np.array([1., 1., 0.]),  # motor 1
        np.array([1., -1., 0.]),  # motor 2
        np.array([1., -1., 0.]),  # motor 3
        np.array([1., 1., 0.]),  # motor 4
    ]  # this debug motor configuration is the same as bvr auv's hovercraft

    bounds = [[-0.9, 0.9]] * 4  # motors can't go outside of (-90%, 90%)...
    deadzone = [[-0.1, 0.1]] * 4  # or inside (-10%, 10%), unless they equal 0 exactly

    degrees = [
        1624.02745,
        874.8296,
        -8224.85246,
        -5033.91631,
        17652.4645,
        12414.2505,
        -20920.4284,
        -17068.0915,
        14947.3121,
        14259.2048,
        -6593.46214,
        -7411.32156,
        1762.97753,
        2365.79253,
        -266.73177,
        -445.54284,
        18.76821,
        49.1195,
        0.5111399,
        0.3424571,
        -0.001137525
    ][::-1]  # this defines our motor's pwm -> thrust curve as t = -0.01 + 0.4p - 0.4p^2 + 1.4p^3

    sim = RoboBoatCore(motor_locations, motor_directions, bounds, deadzone, coefficients=degrees)
    sim_anchovy = AUV(
        motor_controller=MotorController(
            motor_function=sim.motor_interface,
            inertia=InertiaBuilder(
                Cuboid(
                    mass=1,
                    width=4,
                    height=4,
                    depth=0.1,
                    center=np.array([0, 0, 0])
                )).moment_of_inertia(),  # the moment of inertia helps with rotation

            motors=[
                Motor(
                    direction,
                    loc,
                    Motor.Range(bounds[i][0], bounds[i][1]),
                    Motor.Range(deadzone[i][0], deadzone[i][1])
                )
                for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))
            ],
            coefficients=degrees
        ),
        sensors=SensorInterface(sensors=[sim.imu(0.05, 0.05, 0.05, 0.05), sim.gps(1), sim.camera(0)]),
        lock_to_yaw=False,
        clock=sim.clock(),
        map=RoboBoatMap(
            max_velocity=5.0,
            bot_radius=np.sqrt(1.5 ** 2 + 1.0 ** 2),  # max distance from center to motor
            dimensions=((-50, -50), (50, 50)),
            resolution=0.1,
            velocity_std=0.1,
            position_std=3,
            angle_std=0.05,
            rotational_velocity_std=0.05
        )
    )

    # mission = Path(
    #     # AccelerateVector(AccelerationState(Tx=1, local=False), 3),      # start by going right locally,
    #     # AccelerateVector(AccelerationState(Tx=-1, local=False), 3),     # then slow down by going left locally,
    #     # AccelerateVector(AccelerationState(Rz=-20, local=False), 5),    # then spin really fast,
    #     # AccelerateVector(AccelerationState(Tx=-5, local=False), 5),     # then go left globally, while spinning
    #     WaypointTask(3, 0., 0.5, 3, 0., 1, 5, 0, 0, goal=CircleGridObject(2, 8, 0.5))
    # )
    # obstacles = [(5.75838, -3.12832), (-3.87967, -1.95343882)]

    # goals = [(-0.7028, 1.8565), (-6.60901, -2.85482)]
    # goals = [(5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0), (5.0, -5.0)]
    # obstacles = [(-5,0), (0, 5), (5, 0), (0, -5)]

    mission = Path(
        # SleepTask(10)
        EntryGate(
            3, 0., 0.5, 2., 0., 3.0, 3., 0., 0.,
            lookahead_distance=1.5
        )
        # NavigationChannel(
        #     5,
        #     1,
        #     25,
        #     3, 0., 0.5, 2., 0., 3.0, 3, 0, 0.,
        #     lookahead_distance=1.5
        # ),
        # EntryGate(
        #     3, 0., 0.5, 2., 0., 3.0, 3., 0., 0.,
        #     lookahead_distance=1.5,
        #     reverse=True
        # )
        # SpeedChallenge(
        #     3, 0., 0.5, 2., 0., 3.0, 3., 0., 0.,
        #     general_location=np.array([0.0, 0.0]),
        #     lookahead_distance=1.5
        # )
    )
    TELEMETRY.begin_communications(None, "S1", "BEAV")
    sim_anchovy.travel_path(mission, end_telemetry=True)

    # sim_anchovy.travel_path(mission)
    # sim_anchovy.calibrate(Position.ORIGIN)  # set to (0,0,0) at 0 degrees

    # mission = Path(

    # task 1, travel through gate
    # IdentifyGate(Type.ENTRY),
    # TravelGate(),

    # # task 2, navigate channel and circle beacons
    # IdentifyGate(Type.CHANNEL_ENTRANCE),
    # NavigateChannel(),
    # IdentifyBeacons(),
    # CircleGreenBeacons(),

    # # task 3, circle beacon quickly
    # IdentifyGate(Type.SPEED_ENTRANCE),
    # HoldPosition(Position.SPEED_ENTRANCE),
    # IdentifyBeacons(Type.SPEED, needed=1),
    # CircleSpeedBeacon(),

    # # return to home
    # ReturnHome()
    # )

    sim.render()  # this draws an animation using pygame; you can see it in videos/animation.mp4

    # requests.post( # notify when done
    #     "https://ntfy.sh/",
    #     data="Program finished"
    # )
    TELEMETRY.draw_graph(
        [
            lambda data: data["real velocity x"],
            lambda data: data["estimated velocity x"]
        ],
        ["Real Velocity X", "Estimated Velocity X"],
        title="Real vs Estimated Velocity X"
    )
    TELEMETRY.draw_graph(
        [
            lambda data: data["real velocity y"],
            lambda data: data["estimated velocity y"]
        ],
        ["Real Velocity Y", "Estimated Velocity Y"],
        title="Real vs Estimated Velocity Y"
    )
    TELEMETRY.draw_graph(
        [
            lambda data: data["real angular velocity"],
            lambda data: data["estimated angular velocity"]
        ],
        ["Real Angular Velocity", "Estimated Angular Velocity"],
        title="Real vs Estimated Angular Velocity"
    )
    TELEMETRY.draw_graph(
        [
            lambda data: data["loop time"]
        ],
        ["Delta Time"],
        title="Delta Time"
    )
    TELEMETRY.draw_graph(
        [
            lambda data: data["solve time"]
        ],
        ["Solve Time"],
        title="Solve Time"
    )


if __name__ == "__main__":
    main()
