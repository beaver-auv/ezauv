import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from ezauv.communications.communications_handler import CommunicationsHandler
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

    def begin_communications(self, comms, vehicle_id, team_id):
        self.communication_handler = CommunicationsHandler(comms, vehicle_id, team_id)
        self.communication_handler.start_background()

    def submit(self, name, data):
        self.data[name] = data
        self.all_keys.add(name)
    
    def step(self, timestamp):
        entry = {"timestamp": timestamp, **self.data}
        self.telemetry_data.append(entry)
        self.data = {}
        self.current_step += 1

    def build_arrays(self):
        self.built = {}
        for key in self.all_keys:
            self.built[key] = np.array([entry.get(key, np.nan) for entry in self.telemetry_data])

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


TELEMETRY = TelemetryManager()