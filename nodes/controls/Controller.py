#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import numpy as np
import sys
from math import cos, sin, pi, atan2

import tf.transformations
from geometry_msgs.msg import Twist, TransformStamped

from Robot_State import Robot_State, Data
from PID_Controller import PID_controller
from DFL_Controller import DFL_controller
from NLF_Controller import NLF_controller
from MPC_Controller import MPC_controller

class Controller():

	def __init__(self):
		"""initializes the controls node"""
		rospy.loginfo("Server node started.")
	
		# Desired v,degrees
		self.velocity = .25 # m/s
		self.angle = 0 # degrees

		# Controller Gains

		# initializes PID gains
		self.kPID_x = (1,	.001,	.01)
		self.kPID_y = (1,	.001,	.01)
		# initializes Dynamic Feedback Linearization gains
		self.kPD_1 = (0.25,	0.05)
		self.kPD_2 = (0.25,	0.05)
		# initialize Non-Linear Feedback gains
		self.c = (0.5,	1)


		# initializes the robot's position and orientation and time
		self.past_state = None
		self.current_state = None
		self.start = None

		self.area_x = 0
		self.area_y = 0

		self.previous_ = rospy.get_time()

		# options include: "PID" (proportional integral derivative), "DFL" (dynamic feedback linearization), "NLF" (non-linear feedback)
		self.controller = rospy.get_param("cntrllr")

		# Determine the controller:
		if self.controller == "DFL": # dynamic feedback linearization
			self.controller = DFL_controller
		elif self.controller == "NLF": # non-linear feedback
			self.controller = NLF_controller
		elif self.controller == "PID": # PID controller
			self.controller = PID_controller
		elif self.controller == "MPC": # MPC controller
			self.controller = MPC_controller
		else:
			rospy.logerror("Controller Type Unknown. Select DFL, NLF, or PID.")
			sys.exit(0)

		self.moving_avg_count = 5
		self.array_iterator = 0
		self.v_array = [0]*self.moving_avg_count
		self.omega_array = [0]*self.moving_avg_count
		
		# change wand to turtlebot later
		rospy.Subscriber('/vicon/turtlebot_traj_track/turtlebot_traj_track', TransformStamped, self.on_data) 
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, latch=True, queue_size=1)

		rospy.loginfo("rospy.spin()")
		rospy.spin()

	def trajectory_tracking(self, desired_v, desired_theta):
		"""Given a desired velocity and angle, output velocity and angle will be caluclated"""

		# calculate the derivatives of the state (x_dot, y_dot, yaw_dot, etc) and the time step
		state_dot = self.calculate_derivatives()
		
		# desired x,y to track a line (position)
		desired_x = self.start.x + cos(desired_theta)*desired_v*(self.current_state.t-self.start.t)
		desired_y = self.start.y + sin(desired_theta)*desired_v*(self.current_state.t-self.start.t)
		# desired x,y dot (velocity)
		desired_x_dot = desired_v*cos(desired_theta)
		desired_y_dot = desired_v*sin(desired_theta)
		# desired x,y dot dot (acceleration)
		desired_x_dot_dot = 0
		desired_y_dot_dot = 0
		
		# calculate the x,y position error
		error_x = self.calculate_error(self.current_state.x,desired_x)
		error_y = self.calculate_error(self.current_state.y,desired_y)
		# calculate the x,y velocity error
		error_x_dot = self.calculate_error(state_dot.x, desired_x_dot)
		error_y_dot = self.calculate_error(state_dot.y, desired_y_dot)
		
		#calculate the current theta and velocity
		current_theta = self.current_state.yaw

		# get the signed velocity
		#signed_velocity = state_dot.x*cos(current_theta) + state_dot.y*sin(current_theta)
		v_angle = atan2(state_dot.y,state_dot.x)
		# get the sign from the velocity
		#sign = -1 if signed_velocity < 0 else 1 if signed_velocity > 0 else 0
		sign = -1 if v_angle < pi/2.0 or v_angle > pi/2.0 else 1

		# apply sign to pythagorian velocity
		current_v = abs(((state_dot.x)**2+(state_dot.y)**2)**0.5)*sign

		#calculate integrals for PID
		self.area_x += self.calculate_integral(self.current_state.x, self.past_state.x, state_dot.t)
		self.area_y += self.calculate_integral(self.current_state.y, self.past_state.y, state_dot.t)

		#calculate the error in velocity and theta
		error_v = self.calculate_error(current_v, desired_v)
		error_theta = self.calculate_error(current_theta, desired_theta)

		desired_v_dot = 0
		desired_theta_dot = 0
		desired_omega = desired_theta_dot
		
		# Create a Data Wrapper
		data = Data()
		data.current_state_x = self.current_state.x
		data.current_state_y = self.current_state.y
		data.desired_x = desired_x
		data.desired_y = desired_y
		data.current_theta = current_theta
		data.error_x = error_x
		data.error_y = error_y
		data.error_theta = error_theta
		data.error_v = error_v
		data.error_x_dot = error_x_dot
		data.error_y_dot = error_y_dot
		data.desired_v = desired_v
		data.desired_omega = desired_omega
		data.desired_x_dot_dot = desired_x_dot_dot
		data.desired_y_dot_dot = desired_y_dot_dot
		data.time_step = state_dot.t
		data.area_x = self.area_x
		data.area_y = self.area_y
		data.current_theta = current_theta
		data.desired_x_dot = desired_x_dot
		data.desired_y_dot = desired_y_dot
		# runs chosen controller:
		v, omega = self.controller(self, data)

		print "c_x=%.2f m, c_y=%.2f, c_v=%.4f m/s, c_theta=%.2f deg,e_x=%.2f, e_y=%.2f, e_v=%.2f, e_theta=%.2f, v=%.2f, omega=%.2f deg/s" % (self.current_state.x, self.current_state.y, current_v, current_theta*180/pi,error_x, error_y,  error_v, error_theta*180/pi, v, omega*180/pi)
		return (v, omega)


	def calculate_integral(self, current, last, delta_t):
		# calculate difference:
		delta = current - last
		# calculate rectangular area:
		rect_area = current*delta_t
		# calculate triangular area:
		tri_area = delta*delta_t/2.0
		# calculate area under the curve:
		area = rect_area - tri_area
		return area

	def calculate_derivatives(self):
		"""calculates the time derivative of the current stae"""

		delta_state = self.current_state - self.past_state
		# calculates derivatives, where time (t) is the time step
		state_dot = delta_state.divide_by_time()
		return state_dot

	def calculate_error(self, current, goal):
		"""calculates the error between a desired input and a given input"""

		error = goal - current
		return error

	def moving_average(self, new_v, new_omega):
		"""takes in a new velocity and new omega command and returns the average velocity and omega command"""
		self.v_array[self.array_iterator] = new_v
		self.omega_array[self.array_iterator] = new_omega
		v_average = np.mean(self.v_array)
		omega_average = np.mean(self.omega_array)
		self.array_iterator += 1
		if self.array_iterator == self.moving_avg_count:
			self.array_iterator = 0
		return [v_average, omega_average]

	def send_twist_message(self, v, omega):
		"""takes in the velocity and theta"""
		t = Twist()
		v = max(-10, min(10, v))
		omega = max(-10, min(10, omega))
		
		t.linear.x = v
		t.linear.y = 0
		t.linear.z = 0
	 	t.angular.x = 0
		t.angular.y = 0
		t.angular.z = omega
		self.pub.publish(t)


	def on_data(self, data):	
		"""Callback function that handle subscriber data and updates self."""

		# set desired data rate:
		desired_ = .2 # sec
		if ((rospy.get_time() - self.previous_) > desired_):
			self.previous_ = rospy.get_time()	
		else:
			return None
		# sets the position data	
		x = data.transform.translation.x
		y = data.transform.translation.y
		z = data.transform.translation.z
		
		# converts the quaternion to euler angles
		euler = self.quaternion_to_euler(data.transform.rotation)
		
		# sets the orientation data	
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]		
		
		# grabs the current time stamp
		current_time = data.header.stamp.secs + data.header.stamp.nsecs/1E9

		# set states if new state time is unique
		if self.current_state == None:
			self.past_state = self.current_state
			self.current_state = Robot_State(x,y,z,roll,pitch,yaw,current_time)
			return None


		if current_time == self.current_state.t:
			return None

		self.past_state = self.current_state
		self.current_state = Robot_State(x,y,z,roll,pitch,yaw,current_time)

		# runs only once and grabs the beginning state
		if self.start == None:
			self.start = self.current_state

		# calls the trajectory tracker
		if self.past_state == None:
			return None
		
		# sets velocity in m/s:
		velocity = self.velocity
		
		# sets desired theta in radians:
		angle = self.angle*pi/180.0

		# calculates controller:
		v, omega = self.trajectory_tracking(velocity, angle)
		
		# averages recent commands for smooth operation
		#[v_average, omega_average] = self.moving_average(v,omega)

		# sends commands to robot
		self.send_twist_message(v, omega)

	def quaternion_to_euler(self, quaternion):
		"""converts a quaternion to euler angles"""
		#type(pose) = geometry_msgs.msg.Pose
		euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
		return euler

# Main function.
if __name__ == '__main__':
	# Initialize the node and name it.
	print "Initiating server node..."
	rospy.init_node('Controller')
    
	try:
		controller_server = Controller()

	except rospy.ROSInterruptException:
		rospy.logerror("Failed to start server node.")
		pass
