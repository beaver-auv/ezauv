import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from ezauv.communications.communications_handler import CommunicationsHandler
from ezauv.animator import Animator
import socket
from multiprocessing import parent_process

class TelemetryManager:
    def __init__(self):
        self.telemetry_data = []
        self.current_step = 0
        self.data = {}
        self.built = None
        self.all_keys = {"timestamp"}
        self.communication_handler = None
        self.animator = Animator()


    def begin_communications(self, comms, vehicle_id, team_id):
        self.communication_handler = CommunicationsHandler(comms, vehicle_id, team_id)
        self.communication_handler.start_background()
        # position: {copy},
        # rotation: Any,
        # velocity: {copy},
        # motor_positions: Any,
        # motor_accelerations: Any,
        # debug_text: str = "",
        # waypoint_locations: {copy} | None = None,
        # obstacles: Any = None,
        # visible_obstacles: Any = None,
        # obstacle_pixels: Any = None,
        # goal_pixels: Any = None,
        # goal_location:

    def submit(self, name, data):
        self.data[name] = data
        self.all_keys.add(name)
    
    def step(self, timestamp):
        entry = {"timestamp": timestamp, **self.data}
        self.telemetry_data.append(entry)
        self.animator.append(
            timestamp=timestamp,
            position = entry.get("position", None),
            rotation = entry.get("rotation", None),
            velocity = entry.get("velocity", None),
            motor_accelerations = entry.get("accelerations", None),
            debug_text = entry.get("debug text", ""),
            waypoint_locations = entry.get("waypoint locations", None),
            obstacles = entry.get("obstacles", None),
            visible_obstacles = entry.get("visible obstacles", None),
            obstacle_pixels = entry.get("obstacle pixels", None),
            goal_pixels = entry.get("goal pixels", None),
            goal_location = entry.get("goal", None),
        )
        self.data = {}
        self.current_step += 1

    def build_arrays(self):
        self.built = {}
        for key in self.all_keys:
            self.built[key] = np.array([entry.get(key, np.nan) for entry in self.telemetry_data], dtype=object)

    def kill(self):
        print("Stopping telemetry...")
        self.communication_handler.stop_background()
        self.build_arrays()
        print("Telemetry stopped.")

    def draw_graph(self, functions, labels=None, title="Telemetry Graph"):
        fig, ax = plt.subplots()
        if not self.built:
            self.build_arrays()

        x = self.built["timestamp"]
        ys = [func(self.built) for func in functions]

        for y in ys:
            ax.plot(x, y)
        if labels:
            ax.legend(labels)
        ax.set_xlabel("Time (s)")

        plt.savefig("graphs/" + title.lower().replace(" ", "_") + ".png")

    def set_state(self, state=None, position=None, spd_mps=None, heading_deg=None, current_task=None):
        if self.communication_handler:
            self.communication_handler.update_heartbeat(
                state=state,
                position=position,
                spd_mps=spd_mps,
                heading_deg=heading_deg,
                current_task=current_task,
            )

    def send_report(self, report):
        if self.communication_handler:
            self.communication_handler.submit_report(report)

    def animate(self):
        self.animator.render()

TELEMETRY = TelemetryManager()