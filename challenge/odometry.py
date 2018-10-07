#!/usr/bin/python3

import numpy as np
import vrep
import threading
import time

PI = np.pi

# robot
right_wheel_angle = 0
left_wheel_angle = 0
pos_x = 0
pos_y = 0
angle = 0

class odometry(threading.Thread):
	def __init__(self, clientID_, robot_handle_, left_motor_handle_, right_motor_handle_):
		threading.Thread.__init__(self)

		# global clientID, robot_handle, left_motor_handle, right_motor_handle

		self.clientID = clientID_
		self.robot_handle = robot_handle_
		self.left_motor_handle = left_motor_handle_
		self.right_motor_handle = right_motor_handle_

		# from vrep
		self.width = 0.33098
		self.wheel_radius = 0.0975

	def run(self):
		global left_wheel_angle, right_wheel_angle, pos_x, pos_y, angle

		_, position = vrep.simxGetObjectPosition(
			self.clientID, self.robot_handle, -1, vrep.simx_opmode_buffer)
		_, eulerAngles = vrep.simxGetObjectOrientation(
			self.clientID, self.robot_handle, -1, vrep.simx_opmode_buffer)
		_, new_right_wheel_angle = vrep.simxGetJointPosition(
			self.clientID, self.right_motor_handle, vrep.simx_opmode_streaming)
		_, new_left_wheel_angle = vrep.simxGetJointPosition(
			self.clientID, self.left_motor_handle, vrep.simx_opmode_streaming)

		pos_x = position[0]
		pos_y = position[1]
		angle = eulerAngles[2]

		new_left_wheel_angle = (
			new_left_wheel_angle + 2 * PI) % (2 * PI)
		new_right_wheel_angle = (
			new_right_wheel_angle + 2 * PI) % (2 * PI)

		left_wheel_angle = new_left_wheel_angle
		right_wheel_angle = new_right_wheel_angle

		while True:
			self.update()
			time.sleep(0.1)

	def update(self):
		global last_exec_time, left_wheel_angle, right_wheel_angle, pos_x, pos_y, angle

		_, new_right_wheel_angle = vrep.simxGetJointPosition(
			self.clientID, self.right_motor_handle, vrep.simx_opmode_streaming)
		_, new_left_wheel_angle = vrep.simxGetJointPosition(
			self.clientID, self.left_motor_handle, vrep.simx_opmode_streaming)

		if new_left_wheel_angle < 0:
			new_left_wheel_angle = 2 * PI + new_left_wheel_angle

		if new_right_wheel_angle < 0:
			new_right_wheel_angle = 2 * PI + new_right_wheel_angle

		theta_left = new_left_wheel_angle - left_wheel_angle
		theta_right = new_right_wheel_angle - right_wheel_angle

		if(abs(theta_left) > .5):
			if left_wheel_angle > new_left_wheel_angle:
				theta_left = 2 * PI - abs(theta_left)
				if(abs(theta_left) > .5):
					theta_left = 0
				print('case left')
			else:
				theta_left = 0
				
		if(abs(theta_right) > .5):
			if right_wheel_angle > new_right_wheel_angle:
				theta_right = 2 * PI - abs(theta_right)
				if(abs(theta_right) > .5):
					theta_right = 0
				print('case right')
			else:
				theta_right = 0

		left_wheel_angle = new_left_wheel_angle
		right_wheel_angle = new_right_wheel_angle
		delta_l = theta_left * self.wheel_radius
		delta_r = theta_right * self.wheel_radius
		delta_s = (delta_l + delta_r) / 2
		delta_theta = (delta_r - delta_l) / self.width

		pos_x = pos_x + delta_s * np.cos(angle)
		pos_y = pos_y + delta_s * np.sin(angle)
		angle = angle + delta_theta

	def getPos(self):
		global pos_x, pos_y
		return pos_x, pos_y