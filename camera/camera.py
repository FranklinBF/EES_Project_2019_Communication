import cv2
import requests
import numpy as np
import base64
import os
import time
from enum import Enum

class CameraType(Enum):
	PI = 1
	CEILING = 2

class Camera:
    def __init__(self, source=CameraType.PI):
        self.source = source
        self.piHost = None
        self.cap = self._getCap()
        

    def __del__(self):
        if self.cap is not None:
            self.cap.release()

    def _getCap(self):
        cap = None

        if self.source == CameraType.PI:
            if self.piHost is None:
                self.piHost = self._findPiHost()
            url = 'http://{}:8080/?action=stream'.format(self.piHost)

        if self.source == CameraType.CEILING:
            user = 'student'
            password = base64.b64encode('camera')
            url = 'http://{user}:{password}@130.149.234.101:8080/stream/video/mjpeg'.format(user=user, password=password)

        cap = cv2.VideoCapture()
        cap.open(url)
        return cap

    def resetStream(self):
        if self.cap is not None:
            self.cap.release()
        cap = self._getCap()
        self.cap = cap


    def getCurrentImage(self, buffer=False):
        if not buffer:
            self.resetStream()
        ret, frame = self.cap.read()
        return frame if ret else False

    @staticmethod 
    def _findPiHost():
        return "130.149.234.147"
        foundFlag = False
        for i in range(143, 150):
            host = '130.149.234.{}'.format(i)
            print('Trying: {}'.format(host))
            isHostUp = True if os.system('ping -c 1 -W 1 ' + host + '> /dev/null') is 0 else False
            if isHostUp:
                foundFlag = True
                break

        if not foundFlag:
            print('Router not found')
            exit(1)
        return host
