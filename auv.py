import numpy as np
import traceback
import copy
import time
from typing import Callable, List
from scipy.spatial.transform import Rotation as R

from ezauv.hardware.motor_controller import MotorController
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.mission.mission import Path, Subtask
from ezauv.utils import Logger, LogLevel, Clock
from ezauv.map import Map
from ezauv.telemetry import TELEMETRY

class AUV:
    def __init__(self, *,
                 refresh_rate: float = 0.01,            # the rate at which the AUV updates its state
                 motor_controller: MotorController,     # the object to control the motors with
                 sensors: SensorInterface,              # the interface for sensor data
                 pin_kill: Callable = lambda: None,     # an emergency kill function; should disable all motors via pins
                 clock: Clock = Clock(),                # the clock to use for timing

                 logging: bool = False,                 # whether to save log to file
                 console: bool = True,                  # whether to print log to console
                 lock_to_yaw: bool = False,              # whether to lock the AUV to only the yaw rotation axis in global space
                 # more detail for above, this means that if the AUV is pitched/rolled it will 
                 # not account for those rotations when solving for motor commands in global space

                 # unless you plan on rotating strangely, this is highly recommended. if
                 # something goes wrong with the rotation, it can lead to unexpected behavior,
                 # and you really don't want your sub to be stuck rolling in the water

                 # if needed, you can always send manual rotation commands on non-yaw axes which will
                 # avoid this, and this flag can also be enabled/disabled throughout the run
                 map: Map = None
                 ):
        """
        Create a sub wrapper object.\n
        motor_controller: the object to control the motors with\n
        sensors: the interface for all sensor data\n
        pin_kill: an emergency kill function, when the library is having issues. Should manually set motors off\n
        logging: whether to save log to file\n
        console: whether to print log to console
        """
        self.refresh_rate = refresh_rate
        self.motor_controller = motor_controller
        self.sensors = sensors
        self.pin_kill = pin_kill
        self.lock_to_yaw = lock_to_yaw

        self.clock = clock
        self.map = map

        self.logger = Logger(console, logging)

        self.motor_controller.log = self.logger.create_sourced_logger("MOTOR")
        self.sensors.log = self.logger.create_sourced_logger("SENSOR")

        self.logger.log("Sub enabled")
        self.motor_controller.overview()
        self.sensors.overview()

        self.motor_controller.initialize()
        self.sensors.initialize()

        self.subtasks: List[Subtask] = []

    def register_subtask(self, subtask):
        """Register a subtask to be run every iteration for this AUV."""
        subtask.clock = self.clock
        self.subtasks.append(subtask)
        subtask.start(self.map)

    def kill(self):
        """Set all motors to 0 speed, killing the sub."""
        self.motor_controller.set_motors(np.array([0 for _ in self.motor_controller.motors]))

    def travel_path(self, mission: Path, end_telemetry = False) -> None:
        """Execute each Task in the given Path, in order, then kill the sub. Handles errors."""

        self.logger.log("Beginning path")

        try:
            for task in mission.path:
                self.logger.log(f"Beginning task {task.name()}")

                now = self.clock.perf_counter()
                prev_update = now
                task.start(self.map)
                while(not task.finished()):
                    now, prev_update = self.clock.perf_counter(), now
                    dt = now - prev_update
                    sensor_data = self.sensors.get_data()
                    sensor_data["dt"] = dt
                    a = time.perf_counter()
                    self.map.update(sensor_data)
                    wanted_direction = copy.deepcopy(task.wanted_acceleration(self.map))
                    for subtask in self.subtasks:
                        wanted_direction += subtask.update()
                    rotation = sensor_data["rotation"] if "rotation" in sensor_data else R.identity()
                    solve_start = time.perf_counter()
                    solved_motors = self.motor_controller.solve(
                        wanted_direction,
                        rotation,
                        self.lock_to_yaw
                    )
                    TELEMETRY.submit("solve time", time.perf_counter() - solve_start)

                    if(solved_motors[0]):
                        self.motor_controller.set_motors(solved_motors[1])
                    
                    time_till_refresh = self.refresh_rate - (self.clock.perf_counter() - prev_update)
                    # print("Time taken for loop (s):", np.round(self.clock.perf_counter() - prev_update, 4))
                    if(time_till_refresh > 0):
                        self.clock.sleep(time_till_refresh)
                    d = time.perf_counter()
                    TELEMETRY.submit("loop time", self.clock.perf_counter() - prev_update)
                    TELEMETRY.step(self.clock.perf_counter())
                    # if self.clock.perf_counter() - prev_update > 0.3:
                        # print(b - a, c - b, d - c)
                        # raise TimeoutError("Main loop is taking too long (>300ms)")

        except:
            self.logger.log(traceback.format_exc(), level=LogLevel.ERROR)
    
        finally:
            self.logger.log("Killing sub")


            if(not self.motor_controller.killed()):
                kill_methods = [
                ("kill", self.kill),
                # kill through sub interface, uses full library to send kill. should always work

                ("backup kill", self.pin_kill)
                # last resort, directly control pins and send kill commands. doesn't go through library
                # at all, just sends pin commands
                ]
                # when we get more kills (eg hardware kill once we connect it to raspi) add them here
            
                for method_name, method in kill_methods:
                    self.logger.log(f"Attempting {method_name}...")
                    method()
                    if self.motor_controller.killed():
                        self.logger.log(f"{method_name.capitalize()} succeeded")
                        break
                    else:
                        self.logger.log(f"{method_name.capitalize()} failed", level=LogLevel.ERROR)
                else:
                    self.logger.log("All kills ineffective. Manual intervention required", level=LogLevel.ERROR)

            if end_telemetry:
                TELEMETRY.kill()

            self.map.kill()
            self.logger.end()