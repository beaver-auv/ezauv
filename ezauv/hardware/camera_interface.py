# OpenCV (Open computer vision) is the backend for this fantastic code.
import cv2 as cv
from ezauv.utils import LogLevel
import sys


"""
Camera's need to be figured out on thier own. Meaning that if you have multiple cameras, you'll need to figure out which camera is which.
CameraIndex is returned into cv's VideoCapture. This also means that you can specify a video file, not just a camera ID. This can be used for testing.

"""


class CameraObject:
    def __init__(self, log: callable, camera, DivsionFactor):
        self.log = log
        camera = cv.VideoCapture(self.CameraIndex)
        DivisionFactor = self.DivisionFactor

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

    # Only get the frame when the user asks for it. This is MUCH faster then getting every frame.
    def frame(self):
        ret, frame = self.camera.read()
        if not ret:
            self.log("Bro forgot the camera 💀💀💀 ts is not tuff")
            sys.exit(1)
        return frame
