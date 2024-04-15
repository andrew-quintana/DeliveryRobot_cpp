from lib.jetbot.jetbot import Robot
from uuidd import uuid1
import os
import time

class DeliveryRobot:
	def __init__(self):
		self.robot = Robot()

	def take_picture(directory):
		image_path = os.path.join(directory, str(uuid1()) + '.jpg')
		with open(image_path, 'wb') as f:
			f.write(image.value)
			
	def stop():
		self.robot.stop()
		
	def step_forward(speed_m_s=0.4, time_s=0.5):
		self.robot.forward(speed_m_s)
		time.sleep(time_s)
		robot.stop()
		
	def step_backward(speed_m_s=0.4, time_s=0.5):
		self.robot.backward(speed_m_s)
		time.sleep(time_s)
		robot.stop()
		
	def step_left(speed_m_s=0.4, time_s=0.5):
		self.robot.left(speed_m_s)
		time.sleep(time_s)
		robot.stop()
		
	def step_right(speed_m_s=0.4, time_s=0.5):
		self.robot.right(speed_m_s)
		time.sleep(time_s)
		robot.stop()
	
