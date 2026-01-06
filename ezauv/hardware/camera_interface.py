# OpenCV (Open computer vision) is the backend for this fantastic code.
import cv2 as cv
from ultralytics import YOLO
from ezauv.utils import LogLevel
import sys
import socket
import pickle
import struct


"""
Camera's need to be figured out on thier own. Meaning that if you have multiple cameras, you'll need to figure out which camera is which.
CameraIndex is returned into cv's VideoCapture. This also means that you can specify a video file, not just a camera ID. This can be used for testing.

"""


class CameraObject:
    def __init__(
        self,
        log: callable,
        camera,
        DivsionFactor,
        framerate,
        UseModel,
        model,
        IsNetworked,
    ):
        self.log = log

        camera = cv.VideoCapture(self.CameraIndex)
        DivisionFactor = self.DivisionFactor
        framerate = self.framerate
        UseModel = self.UseModel
        model = self.model
        IsNetworked = self.IsNetworked

        cv.set(cv.CAP_PROP_FPS, framerate)

        # If the user chooses to use DivisionFactor to reduce the camera resolution, floor divide the width and height
        if DivisionFactor > 1:
            camera.set(
                cv.CAP_PROP_FRAME_WIDTH,
                int(camera.get(cv.CAP_PROP_FRAME_WIDTH)) // DivisionFactor,
            )
            camera.set(
                cv.CAP_PROP_FRAME_HEIGHT,
                int(camera.get(cv.CAP_PROP_FRAME_HEIGHT)) // DivisionFactor,
            )
        else:
            self.log(
                "WARNING: DivisionFactor is set to 1. This will have a noticable preformace impact.",
                LogLevel=LogLevel.WARNING,
            )

        # This is much faster than the pervious code because frame() is run in a loop in the main code so we only check if the camera is working once.
        # It might seem wierd because the camera could theoreticly fail at any time but this is faster.
        ret, frame = self.camera.read()
        if not ret:
            self.log("ERROR: Can not read camera. The program will now exit.")
            sys.exit(1)

        # Load da model if there is one!
        # We should also find some way to check if the sub is under remote control and if it is then we can unload the model. Clogs needs to get on that RC!
        if UseModel:
            self.log(
                "Loading YOLO model. This will have a preformance impact. Make sure that you computer can keep up with the framerate"
            )
            model = YOLO(str(self.model))
        else:
            self.log("Not using YOLO")

        # If networking is enabled open a port on 6700 for sending the data.
        # I dont see a need for ROS insted because we only need to send just this one thing. At this point ROS just seems like unnecasary middleware.
        if IsNetworked:
            self.s = socket.socket()
            self.s.bind(("localhost", 6700))
            self.log("Opened port on 6700 at addy localhost")
            self.s.listen(5)
            self.log("Socket is listening...\nHang Until client is connected.")
            c, addr = self.s.accept()
            self.log("Connection accepted!")
            self.s.send(b"connected!")

            # Easier to create a second socket for sending the data that the AI gets. Not fucking around with streaming more than two objects
            self.sAI = socket.socket()
            self.sAI.bind(("localhost", 6701))
            self.log("Opened port on 6701 at addy localhost for AI readings")
            self.sAI.listen(5)
            self.log("Socket is listening...\nHang Until client is connected.")
            cAI, addrAI = self.s.accept()
            self.log("Connection accepted!")
            self.sAI.send(b"connected!")

    # Only get the frame when the user asks for it. This is MUCH faster then getting every frame.
    def frame(self):
        ret, frame = self.camera.read()
        if self.IsNetworked:
            frame = pickle.dumps(frame)
            msg_size = struct.pack("I", len(frame))
            self.s.sendall(msg_size + frame)
        else:
            return frame

    # Same thing but allow loading in a YOLO model. By adding this as another function we can skip a theoretical if check in the above.
    # Also load the module in __init__ because this is much faster than loading and unloading the model every time in one fuction
    def frame_yolo(self):
        ret, frame = self.camera.read()
        results = self.model(frame)
        if self.IsNetworked:
            frame = pickle.dumps(frame)
            results = pickle.dumps(results)
            msg_size = struct.pack("I", len(frame))
            msg_results_size = struct.pack("I", len(results))
            # Send EVERYTHING!
            self.s.sendall(msg_size + frame)
            self.sAI.sendall(msg_results_size + results)
        else:
            return frame, results

    def bye(self):
        self.s.close()
        self.camera.release()


# Function to paruse the data recived
def recv_exact(sock, size):
    # Recive exact size
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data


# CameraRecivingClass
# This shit is so bad it makes my dick hurt


class CameraRemote:
    def __init__(self, log: callable, host, port, IsAI):
        self.log = log

        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((host, port))
        self.client.listen(10)
        self.plug, self.addr = self.client.accept()

        if IsAI:
            self.clientAI = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.clientAI.connect((host, int(port) + 1))
            self.clientAI.listen(10)
            self.plugAI, self.addrAI = self.clientAI.accept()

    def RecivePlain(self):
        msg_size = self.client.recv(4)
        if not msg_size:
            sys.exit(0)
        msg_size = struct.unpack("I", msg_size)[0]
        frame = recv_exact(self.plug, msg_size)
        return frame

    def ReciveAI(self):
        msg_size = self.client.recv(4)
        if not msg_size:
            sys.exit(0)
        msg_size = struct.unpack("I", msg_size)[0]
        frame = recv_exact(self.plug, msg_size)

        msg_ai_size = self.clientAI.recv(4)
        if not msg_ai_size:
            sys.exit(0)
        msg_size = struct.unpack("I", msg_ai_size)[0]
        results = recv_exact(self.plugAI, msg_size)

        return frame, results
