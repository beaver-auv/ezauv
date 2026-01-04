# OpenCV (Open computer vision) is the backend for this fantastic code.
import cv2 as cv
from ultralytics import YOLO
from ezauv.utils import LogLevel
import sys


"""
Camera's need to be figured out on thier own. Meaning that if you have multiple cameras, you'll need to figure out which camera is which.
CameraIndex is returned into cv's VideoCapture. This also means that you can specify a video file, not just a camera ID. This can be used for testing.

"""


class CameraObject:
    def __init__(
        self, log: callable, camera, DivsionFactor, framerate, UseModel, model
    ):
        self.log = log
        camera = cv.VideoCapture(self.CameraIndex)
        DivisionFactor = self.DivisionFactor
        framerate = self.framerate
        UseModel = self.UseModel
        model = self.model

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

    # Only get the frame when the user asks for it. This is MUCH faster then getting every frame.
    def frame(self):
        ret, frame = self.camera.read()
        return frame

    # Same thing but allow loading in a YOLO model. By adding this as another function we can skip a theoretical if check in the above.
    # Also load the module in __init__ because this is much faster than loading and unloading the model every time in one fuction
    def frame_yolo(self):
        ret, frame = self.camera.read()
        results = self.model(frame)
        return frame, results
