import numpy as np
import time
import socket
import re
from copy import deepcopy
import signal
import sys
import os
from scipy.interpolate import CubicSpline


from .tcp_class import tcp_communication
import matplotlib.pyplot as plt

#from .JointAngleMixing import MotorArmMixing #accounts for coupling in motion
#from .getRobotPose import getOptitrackPose



"""
Example setup

motors = Motors()
motors.tcp_init(socket_ip, socket_port)
motors.arm_motors()

#Zeros the arm to home position
motors.zero_arm(track_data, NatNet)

"""

class Motors():
	def __init__(self, encoder_counts, gear_ratio, P, I, D, maxVelocity, maxPosition):
		#Constants -> also belt pitch 2mm
		self.encoder_counts = encoder_counts #counts per revolution measured
		self.gear_ratio = gear_ratio#x:1, x is gear ratio
		self.P = P
		self.I = I
		self.D = D
		self.maxVelocity = maxVelocity
		self.maxPosition = maxPosition
		
		self.trajPlanner = trajectoryGenerator()

		self.counts_per_revolution = self.gear_ratio * self.encoder_counts
		self.counts_per_radian = self.counts_per_revolution / (2 * np.pi)
		self.counts_per_degree = self.counts_per_revolution / 360

		self.motor_pos_commanded = np.zeros(8)	#running track of motor position in encoder counts, updates with each command

		self.motor_encoders_data = np.zeros(8) #running track of read data
		self.limit_switches_data = np.zeros(8)
		self.joint_encoders_data = np.zeros(4) #running track of read data
		self.avg_current = np.zeros(8)

		self.current_time = time.time()
		self.time_last_run = time.time()
		self.error_cum = 0
		self.command_time = 0

		#communication stuff setup on call to tcp_init
		self.tcp = None
		self.client_socket = None
		self.zero_position = np.zeros(8)#None
		self.dt = 0.005


	def tcp_init(self, socket_ip, socket_port):
		self.tcp = tcp_communication(socket_ip, socket_port)
		self.client_socket = self.tcp.open_socket()
		self.client_socket.settimeout(1.0) #THIS IS NEW 1/17/2019 and may cause issues
		IsWindows = os.name == 'nt'
		if IsWindows:
			self.tcp.setpriority()

	def arm_motors(self):
		self.read_buff()
		print("initializing motors to {}".format(self.motor_encoders_data))
		#self.zero_position = deepcopy(self.motor_pos_commanded) #removed 1/17/2018, we nolonger save zeros to text file, fix runaway?
		time.sleep(1)
		self.read_buff()
		data = ('b' + 'arm' + 'd')
		print("Arming motors")
		self.client_socket.send(data.encode())
		self.read_buff()
		self.command_motors(self.motor_encoders_data) #added 1/17/2019
		time.sleep(0.25)
		return

	def command_motors(self, position): #sends new positions to controller and updates local position with encoder reads
		self.motor_pos_commanded = position
		pos = self.motor_pos_commanded + self.zero_position
		data = ('b'+ str(int(pos[0])) + ' ' + str(int(pos[1])) + ' ' + str(int(pos[2])) + ' ' +
					 str(int(pos[3])) + ' ' + str(int(pos[4])) + ' ' + str(int(pos[5])) + ' ' +
					 str(int(pos[6])) + ' ' + str(int(pos[7])) + 'd')
		self.client_socket.send(data.encode())
		self.read_buff()
		return

	def read_buff(self, print_sensors = False):
		try:
			data = str(self.client_socket.recv(1024))
			#print(data)
			data = re.split('\s', data)
			if data[1] == 'closeports':
				self.client_socket.close()
				print("\n\nServer side closed. Closing ports now.\n\n")
				sys.exit()
			if data[1] == 'err':
				print("*** C side has an error or needs to be armed ***\n")
			else:
				self.motor_encoders_data = np.array(list(map(int, data[1:9])))
				self.limit_switches_data = np.array(list(map(int, data[9:17])))
				self.joint_encoders_data = np.array(list(map(int, data[17:21])))
				self.avg_current = np.array(list(map(float, data[21:29])))
				self.command_time = int(data[29])
				#print(data)
			if print_sensors:
				print('Read motor encoder positions {}'.format(self.motor_encoders_data))
				print('Read joint encoder positions {}'.format(self.joint_encoders_data))
				#print('Read limit switch values {}'.format(self.limit_data))
		except:
			print('We had a timeout (probably)?')
		return

	def tcp_close(self):
		data = ('b'+ 'stop' +'d')
		self.client_socket.send(data.encode())
		self.client_socket.shutdown(socket.SHUT_RDWR)
		self.client_socket.close()
		return

	def zero_motors_radians(self, axis, position):
		self.zero_position[axis] = position * self.counts_per_radian + self.zero_position[axis]
		return

	def command_motors_radians(self, positions):
		self.command_motors(positions * self.counts_per_radian)
		return

	def run_trajectory(self, setpoints, velocity):
		if (setpoints.all() > self.maxPosition[0]) and (setpoints.all() < self.maxPosition[1]) and velocity.all() < self.maxVelocity:
			trajectory1, _ = self.trajPlanner.createTrajectoryMaxVelocity(self.get_motors_position_radians(), setpoints, velocity, self.dt)
			#rajectory1 = self.trajPlanner.createTrajectoryNumPoints(self.get_motors_position_radians(), setpoints, 50
			for i in range(trajectory1.shape[1]):
			    setpoint = list(trajectory1[:,i])
			    self.command_motors_radians((trajectory1[:,i]))
			    time.sleep(self.dt)
			return 0
		else:
			print("this is out of range")
			return -1
	def get_motors_position_radians(self):
		return (self.motor_encoders_data - self.zero_position) / self.counts_per_radian

	def get_motors_setpoint_radians(self):
		return (self.motor_pos_commanded) / self.counts_per_radian

class CableReduction(Motors):
	def __init__(self):
		super(CableReduction, self).__init__(encoder_counts=2000, gear_ratio=20, P = 5, I = 0, D = 0, maxVelocity = np.pi, maxPosition = [-7/16*np.pi, 7/16*np.pi])

class maxonGearSmall(Motors):
	def __init__(self):
		super(maxonGearSmall, self).__init__(encoder_counts=1440, gear_ratio=479, P = 5, I = 0, D = 0, maxVelocity = np.pi, maxPosition = [-20*np.pi, 20*np.pi])
        
class maxonLarge(Motors):
	def __init__(self):
		super(maxonLarge, self).__init__(encoder_counts=20000, gear_ratio=1, P = 5, I = 0, D = 0, maxVelocity = np.pi, maxPosition = [-20*np.pi, 20*np.pi])


class trajectoryGenerator:
    
    #list of start points and end points and corresponding max velocities,
    #period is in how fast the output command is sent to the robot, unit should be same as velocity
    def createTrajectoryMaxVelocity(self, startPoints, endPoints, maxVelocities, period):
        
        startPoints = np.array(startPoints)
        endPoints = np.array(endPoints)
        maxVelocities = np.array(maxVelocities)
        
        dist = np.abs(endPoints - startPoints)
        
        #cubic spline has maximum velocity at the center (1.5 distance/sample)
        numPoints_list = (np.ceil(1.5*(dist/(period*maxVelocities)))).astype(int)
                        
        trajectory = np.ones((len(startPoints), np.max(numPoints_list)))
        time = np.linspace(0, np.max(numPoints_list)*period, np.max(numPoints_list))
                
        for i in range(0, len(startPoints)):
            trajectory[i, :numPoints_list[i]] = self.createTrajectoryNumPoints(startPoints[i], endPoints[i], numPoints_list[i])
            trajectory[i, numPoints_list[i]:] = endPoints[i]
            
        return trajectory, time
    
    def createTrajectoryNumPoints(self, startPoints, endPoints, num_points):
        
        startPoints = np.array(startPoints)
        endPoints = np.array(endPoints)
        
        #Create smooth function with cubic spline
        # y = -2x^3 + 3x^2
        x = np.linspace(0, 1, num_points)
        y = (-2*x**3 + 3*x**2)
        
        dist = endPoints - startPoints
        if dist.size > 1:
            numDim = len(dist)
        else:
            numDim = 1
            
        trajectory = np.ones((numDim, num_points))
        
        if numDim > 1:
            for i in range(0, numDim):
                trajectory[i, :] = dist[i]*y + startPoints[i]
        else:
            trajectory[0, :] = dist*y + startPoints
        ''' 
        if num_points > 0:
            x = np.array([0,num_points])
            #print(x)

            y = np.array([startPoints,endPoints])

            f = CubicSpline(x, y, bc_type = 'clamped')

            x_new = np.linspace(0, num_points, num=num_points, endpoint=True)
            y_new = f(x_new)
        else:
            y_new = endPoints'''
            
        return trajectory
        
    def plotTrajectory(self, trajectory, time = None):
        for i in range(0, trajectory.shape[0]):
            if time is not None:
                plt.plot(time, trajectory[i,:])
            else:
                plt.plot(trajectory[i,:])
        plt.show()