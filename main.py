import cv2
import numpy as np
import time
from camera.camera import Camera, CameraType
from camera.video import Video, Videos
from ImageProcessing.roadAnalyzer_birdeye import RoadAnalyzer
from client import start_connection_with_nxt
from communication.message import Message
from PIL import Image
SEND_PERIOD = 0.03 # time interval for data sent to nxt
SEND_ANGLE = True  # *for testing purposes*


#Set Up
#stream = Video(Videos.OBSTACLES)	# *for testing purposes*
stream = Camera(CameraType.PI)
roadAnalyzer = RoadAnalyzer()	# intialize Class

if SEND_ANGLE:
	com = start_connection_with_nxt()	# setup connection


# Read Captured camera feed
start = time.time() 
while(True):
	
	frame = stream.getCurrentImage()
	
	# Image Processing

	
	drivingAngle,operation_flag,img_final= roadAnalyzer.getAngle(frame)

		
	# Display Images
	cv2.putText(frame,'Driving Angle: {}'.format(-drivingAngle),(0,600), cv2.FONT_HERSHEY_SIMPLEX, 0.8,(100,100,255),2,cv2.LINE_AA)
	cv2.imshow('Raw Image', frame)
	cv2.imshow('Proccessed Image', img_final)
	print('FLAG: ', operation_flag, ' Radiant: ', drivingAngle)

	# Send to NXT
	if SEND_ANGLE:
		if time.time() > start + SEND_PERIOD:
			msg = Message(flags=operation_flag, payload=drivingAngle)
			msg.send(com.s)
			start = time.time()
			if operation_flag == Message.HEADER_TURN_AROUND:
				time.sleep(2)
				roadAnalyzer = RoadAnalyzer()
		#receive flag and check if image processing needs to be resetted
			

	''' Press Q on keyboard to  exit'''
	if cv2.waitKey(25) & 0xFF == ord('q'):
		break

# Closes all the frames
cv2.destroyAllWindows()
