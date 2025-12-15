import numpy as np
import traceback
import copy
from typing import Callable, List
from scipy.spatial.transform import Rotation as R

from ezauv.hardware.motor_controller import MotorController
from ezauv.hardware.sensor_interface import SensorInterface
from ezauv.mission.mission import Path, Subtask
from ezauv.utils import Logger, LogLevel, Clock

class AUV:
    def __init__(self, *,
                 refresh_rate: float = 0.01,            # the rate at which the AUV updates its state
                 motor_controller: MotorController,     # the object to control the motors with
                 sensors: SensorInterface,              # the interface for sensor data
                 pin_kill: Callable = lambda: None,     # an emergency kill function; should disable all motors via pins
                 clock: Clock = Clock(),                # the clock to use for timing

                 logging: bool = False,                 # whether to save log to file
                 console: bool = True,                  # whether to print log to console
                 lock_to_yaw: bool = False              # whether to lock the AUV to only the yaw rotation axis in global space
                 # more detail for above, this means that if the AUV is pitched/rolled it will 
                 # not account for those rotations when solving for motor commands in global space

                 # unless you plan on rotating strangely, this is highly recommended. if
                 # something goes wrong with the rotation, it can lead to unexpected behavior,
                 # and you really don't want your sub to be stuck rolling in the water

                 # if needed, you can always send manual rotation commands on non-yaw axes which will
                 # avoid this, and this flag can also be enabled/disabled throughout the run
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

    def kill(self):
        """Set all motors to 0 speed, killing the sub."""
        self.motor_controller.set_motors(np.array([0 for _ in self.motor_controller.motors]))

    def travel_path(self, mission: Path) -> None:
        """Execute each Task in the given Path, in order, then kill the sub. Handles errors."""

        self.logger.log("Beginning path")

        try:
            for task in mission.path:
                self.logger.log(f"Beginning task {task.name()}")
                task.clock = self.clock

                
                while(not task.finished()):
                    prev_update = self.clock.perf_counter()
                    sensor_data = self.sensors.get_data()
                    wanted_direction = copy.deepcopy(task.update(sensor_data))
                    
                    for subtask in self.subtasks:
                        wanted_direction += subtask.update(sensor_data)

                    rotation = sensor_data["rotation"] if "rotation" in sensor_data else R.identity()
                    solved_motors = self.motor_controller.solve(
                        wanted_direction,
                        rotation,
                        self.lock_to_yaw
                    )

                    if(solved_motors[0]):
                        self.motor_controller.set_motors(solved_motors[1])

                    time_till_refresh = self.refresh_rate - (self.clock.perf_counter() - prev_update)
                    if(time_till_refresh > 0):
                        self.clock.sleep(time_till_refresh)

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
            
            self.logger.end()