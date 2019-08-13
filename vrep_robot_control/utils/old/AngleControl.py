import numpy as np
import time
import os
import socket
from copy import deepcopy

from .JointAngleMixing import MotorArmMixing #accounts for coupling in motion
from .motor_class import motors
from .tcp_class import tcp_communication
from .getRobotPose import getOptitrackPose

"""
This class sets up the motors. It then takes in joint angle commands and accounts for 
the mixing that will occur due to cable coupling. It will run PID control on the values 
inputed and directly command the motors.
"""

"""
Example usage of how to setup a motorcontrol class:
socket_ip = '192.168.1.39'
socket_port = 1122

P=0
I=0
D=0

joint_motor_indexes = [0,1,2,3]

MC = MotorControl(P,I,D,joint_motor_indexes, control_freq = 20)
MC.tcp_init(socket_ip, socket_port)
MC.motor_init()

print("Arming motors now...")
MC.motors.arm()
time.sleep(2)
error_cum = 100 #initialize break error to a large value

MC.zero_arm(j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler)

***
setup optitrak here
***

MC.zero_arm()

"""

class MotorControl():

	def __init__(self,P ,PL ,I, IL ,D, joint_motor_indexes, control_freq = 50): #gain values for joint level control
		#Constants
		self.encoder_counts = 1440
		self.gear_ratio = 470
		self.counts_per_revolution = self.gear_ratio * self.encoder_counts
		self.counts_per_radian = self.counts_per_revolution / (2 * np.pi)
		self.counts_per_degree = self.counts_per_revolution / 360
		self.counter = 0

		#Values passed in
		self.P = P
		self.PL = PL
		self.IL = IL
		self.I = I
		self.D = D
		self.error_cum = 0
		self.joint_motor_indexes = joint_motor_indexes
		self.motor_command = np.zeros(8)
		self.control_freq = control_freq
		self.time_last_run = time.time()
		self.current_time = time.time()

		#Values set by user
		self.tcp = None
		self.my_socket = None
		self.motors = None
		self.zero_position = None

	def tcp_init(self, socket_ip, socket_port):
		self.tcp = tcp_communication(socket_ip, socket_port)
		self.my_socket = self.tcp.open_socket()
		IsWindows = os.name == 'nt'
		if IsWindows:
			self.tcp.setpriority()

	def tcp_close(self):
		data = ('b'+ 'stop' +'d')
		self.my_socket.send(data.encode())
		self.my_socket.shutdown(socket.SHUT_RDWR)
		self.my_socket.close()

	def motor_init(self, dt=0.01): #dt is only used for the internal sine function for motor testing
		self.motors = motors(CLIENT_SOCKET = self.my_socket, dt = dt, step_size = 100, degrees_count_motor = 1./self.counts_per_revolution, degrees_count_motor_joint = 1)
		self.motors.read_buff() 						#c side sends out initial stored encoder positions, grabs them
		print("initializing motors to {}".format(self.motors.motor_encoders_data))
		time.sleep(1)
		self.motors.read_buff()
		self.motors.motor_pos = deepcopy(self.motors.motor_encoders_data) 	#initialize to stored encoder positions
		self.zero_position = deepcopy(self.motors.motor_pos)
		self.motors.command_motors(self.motors.motor_pos)			#echo read value, values arent used since c side is in err mode


	def zero_arm(self, track_data, NatNet):	
		k = 0.025
		kl = 0.1
		motor_command = deepcopy(self.zero_position)

		#---------------------------------------#---------------------------------------
		#Zeroing loop                           #---------------------------------------
		#---------------------------------------#---------------------------------------
		j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler = getOptitrackPose(track_data, NatNet)

		retractFlag = 0

		while np.abs(j1_angle) > 1 or np.abs(j2_angle) > 1 or np.abs(j3_angle) > 1 or np.abs(j4_pos - 175) > 0.2:

			j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler = getOptitrackPose(track_data, NatNet)

			if retractFlag == 0:
				while np.abs(j4_pos - 175) > 0.2:
					motor_command[self.joint_motor_indexes[3]] += int(-1* (j4_pos-175) * self.counts_per_degree * kl)
					self.motors.command_motors(motor_command)
					print("Current joint positions: \n j1: {}\n j2: {}\n j3: {}\n j4: {}\n".format(j1_angle, j2_angle, j3_angle, j4_pos))
					print("Motor command: {}".format(motor_command))
					j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler = getOptitrackPose(track_data, NatNet)
					time.sleep(0.05)

				retractFlag = 1

			if np.abs(j1_angle) > 1:
				motor_command[self.joint_motor_indexes[0]] += int(-1* j1_angle * self.counts_per_degree * k)
			elif np.abs(j2_angle) > 1:
				motor_command[self.joint_motor_indexes[1]] += int(j2_angle * self.counts_per_degree * k)
			elif np.abs(j3_angle) > 1:
				motor_command[self.joint_motor_indexes[2]] += int(j3_angle * self.counts_per_degree * k)
			elif np.abs(j4_pos - 175) > 0.2:
				motor_command[self.joint_motor_indexes[3]] += int(-1* (j4_pos-175) * self.counts_per_degree * kl)
			error_cum = np.abs(j1_angle) + np.abs(j2_angle) + np.abs(j3_angle)

			print("Current joint positions: \n j1: {}\n j2: {}\n j3: {}\n j4: {}\n".format(j1_angle, j2_angle, j3_angle, j4_pos))
			print("Motor command: {}".format(motor_command))

			self.motors.command_motors(motor_command)

			time.sleep(0.05)

		self.zero_position = deepcopy(motor_command)


	def update(self, current_angles, angle_setpoints, trajectory = None, print_data=False):
		#Radians
		#returns a flag that indicates if the update was run or not
		self.current_time = time.time()

		error = angle_setpoints - current_angles
		arm_angles_signal = np.zeros((4,1))

		if self.current_time - self.time_last_run >= 1/self.control_freq:

			dt = self.current_time - self.time_last_run
			self.error_cum = error * dt + self.error_cum
			arm_angles_signal[0] = angle_setpoints[0] + error[0] * self.P + self.error_cum[0] * self.I
			arm_angles_signal[1] = angle_setpoints[1] + error[1] * self.P + self.error_cum[1] * self.I
			arm_angles_signal[2] = angle_setpoints[2] + error[2] * self.P + self.error_cum[2] * self.I
			arm_angles_signal[3] = angle_setpoints[3] + error[3] * self.PL + self.error_cum[3] * self.IL

			motor_angle_setpoints = MotorArmMixing(arm_angles_signal) #4x1 matrix return

			self.motor_command[self.joint_motor_indexes[0]] = self.zero_position[self.joint_motor_indexes[0]] + motor_angle_setpoints[0] * self.counts_per_radian
			self.motor_command[self.joint_motor_indexes[1]] = self.zero_position[self.joint_motor_indexes[1]] + motor_angle_setpoints[1] * self.counts_per_radian * -1
			self.motor_command[self.joint_motor_indexes[2]] = self.zero_position[self.joint_motor_indexes[2]] + motor_angle_setpoints[2] * self.counts_per_radian * -1
			self.motor_command[self.joint_motor_indexes[3]] = self.zero_position[self.joint_motor_indexes[3]] + motor_angle_setpoints[3] * self.counts_per_radian

			#for i in range(len(joint_motor_indexes))
			#	self.motor_command[joint_motor_indexes[i]] = self.zero_position + motor_angle_setpoints[i] * self.counts_per_radian
			if print_data and self.counter % 100 == 0:
				#Converting to degrees
				# print("ERROR", error)
				# print("setpoints", angle_setpoints)
				# print("current angles", current_angles)
				# print(self.motor_command)
				print("ERROR", np.append(error[0:-1] * 180/np.pi, error[-1]))
				print("cum error for I", self.error_cum)
				print("setpoints", np.append(angle_setpoints[0:-1] * 180/np.pi, angle_setpoints[-1]))
				print("current angles", np.append(current_angles[0:-1] * 180/np.pi, current_angles[-1]))
				print(self.motor_command.astype(int))
				print("\n")
			
			self.counter = self.counter + 1
			self.motors.command_motors(self.motor_command.astype(int))

			self.time_last_run = self.current_time
			update = 1
		else:
			update = 0

		return update



#self regulating of control time
#set loop speed