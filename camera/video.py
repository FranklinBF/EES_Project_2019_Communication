from enum import Enum
import os
import cv2

class Videos(Enum):
    OBSTACLES = "with_obs.mp4"
    PLANE = "without_obs.mp4"

class Video:
    def __init__(self, source=Videos.OBSTACLES, cycle=True):
        self.source = os.path.join(os.path.split(os.path.abspath(__file__))[0],"Videos", source.value)
        self.cap = self._getCap()
        self.cycle = cycle
        
    def _getCap(self):
        cap = cv2.VideoCapture(self.source)
        return cap

    def resetVideo(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = self._getCap()
    
    def getCurrentImage(self):
        ret, frame = self.cap.read()
        if ret:
            return frame
        # Restart Video 
        if self.cycle:
            self.resetVideo()
            ret, frame = self.cap.read()
            return frame
        return False
