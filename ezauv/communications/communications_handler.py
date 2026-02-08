from datetime import datetime, timezone
from queue import Queue, Empty
from threading import Thread, Event, Lock
import time
from google.protobuf.timestamp_pb2 import Timestamp
from ezauv.communications.report_pb2 import *
from struct import pack

MAX_PAYLOAD_LEN = 255

class CommunicationsHandler:
    def __init__(self, comms, vehicle_id, team_id):
        self.comms = comms
        self.seq = 0
        self.vehicle_id = vehicle_id
        self.team_id = team_id
        self._queue = Queue()
        self._stop_event = Event()
        self._thread = None
        self._heartbeat_lock = Lock()
        self._send_lock = Lock()
        self._heartbeat_state = {
            "state": RobotState.STATE_UNKNOWN,
            "position": None,
            "spd_mps": 0.0,
            "heading_deg": 0.0,
            "current_task": TaskType.TASK_UNKNOWN,
        }

    def send_report(self, report):
        if not isinstance(report, Report):
            raise TypeError("report must be a Report message")

        with self._send_lock:
            print("Beginning to send report...")
            report.vehicle_id = self.vehicle_id
            report.team_id = self.team_id
            report.seq = self.seq

            sent_at = Timestamp()
            sent_at.FromDatetime(datetime.now(timezone.utc))
            report.sent_at.CopyFrom(sent_at)

            payload = report.SerializeToString()
            if len(payload) > MAX_PAYLOAD_LEN:
                raise ValueError(f"payload length {len(payload)} exceeds {MAX_PAYLOAD_LEN} bytes")

            frame = b"$R" + pack("!B", len(payload)) + payload + b"!!"
            # self.comms.sendAll(frame)
            print("Sending report:", report)
            self.seq += 1


    def send_heartbeat(
        self,
        state=RobotState.STATE_UNKNOWN,
        position=None,
        spd_mps=0.0,
        heading_deg=0.0,
        current_task=TaskType.TASK_UNKNOWN,
    ):
        report = Report()
        report.heartbeat.state = state
        if position is not None:
            report.heartbeat.position.CopyFrom(_coerce_latlng(position))
        report.heartbeat.spd_mps = spd_mps
        report.heartbeat.heading_deg = heading_deg
        report.heartbeat.current_task = current_task
        self.send_report(report)

    def update_heartbeat(self, state=None, position=None, spd_mps=None, heading_deg=None, current_task=None):
        with self._heartbeat_lock:
            if state is not None:
                self._heartbeat_state["state"] = state
            if position is not None:
                self._heartbeat_state["position"] = position
            if spd_mps is not None:
                self._heartbeat_state["spd_mps"] = spd_mps
            if heading_deg is not None:
                self._heartbeat_state["heading_deg"] = heading_deg
            if current_task is not None:
                self._heartbeat_state["current_task"] = current_task

    def submit_report(self, report):
        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("background sender is not running")
        self._queue.put(report)

    def start_background(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop_background(self, join_timeout_s=2.0):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)
            self._thread = None

    def _run(self):
        next_heartbeat = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_heartbeat:
                with self._heartbeat_lock:
                    hb = dict(self._heartbeat_state)
                self.send_heartbeat(
                    state=hb["state"],
                    position=hb["position"],
                    spd_mps=hb["spd_mps"],
                    heading_deg=hb["heading_deg"],
                    current_task=hb["current_task"],
                )
                next_heartbeat = now + 1.0

            timeout = max(0.0, next_heartbeat - time.monotonic())
            try:
                report = self._queue.get(timeout=timeout)
            except Empty:
                continue

            self.send_report(report)


def _coerce_latlng(position):
    if isinstance(position, LatLng):
        return position
    if isinstance(position, (tuple, list)) and len(position) == 2:
        latlng = LatLng()
        latlng.latitude = float(position[0])
        latlng.longitude = float(position[1])
        return latlng
    raise TypeError("position must be a LatLng or (latitude, longitude)")
