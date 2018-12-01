#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import Queue
import numpy as np

from Robot_State import Robot_State
from std_msgs.msg import Empty, String
import tf.transformations
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from PID_Controller import PID_controller

class Controller():

	def __init__(self):
		"""initializes the controls node"""
		rospy.loginfo("Server node started.")
	
		#initializes the robot's position and orientation and time
		self.past_state = None
		self.current_state = None

		#initializes PID gains
		self.kp_v = 1
		self.ki_v = 0
		self.kd_v = .01
		self.kp_theta = 1
		self.ki_theta = 0
		self.kd_theta = .01

		#initializes Dynamic Feedback Linearization gains
		self.kp_1 = 1
		self.kp_2 = 1
		self.kd_1 = 0.2
		self.kd_2 = 0.2
		

		#options include: "PID" (proportional integral derivative), "DFL" (dynamic feedback linearization), "NLF" (non-linear feedback)
		#self.controller = "PID"
		self.controller = "DFL"
		#self.controller = "NLF"
		
		self.moving_avg_count = 5
		self.array_iterator = 0
		self.v_array = [0]*self.moving_avg_count
		self.theta_array = [0]*self.moving_avg_count
	
		#change wand to turtlebot later
		rospy.Subscriber('/vicon/turtlebot/turtlebot', TransformStamped, self.on_data) 
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, latch=True, queue_size=1)

		if SIMULATION == True:
			rospy.Subscriber('/tf', TFMessage, self.on_tf) 
			self.pub_tf = rospy.Publisher('/vicon/turtlebot/turtlebot', TransformStamped, latch=True, queue_size=1)

		rospy.spin()

	def trajectory_tracking(self, desired_v, desired_theta):
		#rospy.loginfo("traj tracking")

		state_dot = self.calculate_derivatives()
		desired_x = self.past_state.x + cos(self.past_state.yaw)*desired_v*self.state_dot.t
		desired_y = self.past_state.y + sin(self.past_state.yaw)*desired_v*self.state_dot.t
		desired_x_dot = desired_v
		desired_y_dot = 0
		desired_x_dot_dot = 0
		desired_y_dot_dot = 0
		error_x = self.calculate_error(self.current_state.x,desired_x)
		error_y = self.calculate_error(self.current_state.y,desired_y)
		error_x_dot = self.calculate_error(self.state_dot.x, desired_x_dot)
		error_y_dot = self.calculate_error(self.state_dot.y, desired_y_dot)
		current_v = ((state_dot.x)**2+(state_dot.y)**2)**0.5
		error_v = self.calculate_error(current_v, desired_v)
		current_theta = self.current_state.yaw
		error_theta = self.calculate_error(current_theta, desired_theta)
		
		#runs chosen controller:
		if self.controller == "DFL": #dynamic feedback linearization
			v, theta = DFL_controller(self, error_x, error_y, error_x_dot, error_y_dot, desired_x_dot_dot, desired_y_dot_dot, desired_v, current_theta)
		else if self.controller == "NLF": #non-linear feedback
			v, theta = NLF_controller(self, error_v, error_theta)
		else: #defaults to PID controller
			v, theta = PID_controller(self, error_v, error_theta)
		return (v, theta)

	def calculate_derivatives(self):
		#rospy.loginfo("calc derivs")
		delta_state = self.current_state - self.past_state
		#calculates derivatives, where time (t) is the time step
		state_dot = delta_state.divide_by_time()
		return state_dot

	def calculate_error(self, current, goal):
		#rospy.loginfo("calc error")
		error = current - goal
		return error

	def moving_average(self, new_v, new_theta):
		#takes in a new velocity and new theta command and returns the average velocity and theta command
		self.v_array[self.array_iterator] = new_v
		self.theta_array[self.array_iterator] = new_theta
		v_average = np.mean(self.v_array)
		theta_average = np.mean(self.theta_array)
		self.array_iterator += 1
		if self.array_iterator == self.moving_avg_count:
			self.array_iterator = 0
		return [v_average, theta_average]

	def send_twist_message(self, v, theta):
		"""takes in the velocity and theta"""
		rospy.loginfo("linear x: %.4f angular z: %.4f" % (v, theta))
		t = Twist()
		t.linear.x = -v
		t.linear.y = 0
		t.linear.z = 0
	 	t.angular.x = 0
		t.angular.y = 0
		t.angular.z = theta
		self.pub_tf.publish(t)


	def on_data(self, data):
		#rospy.loginfo("on data")
		
		"""Callback function that handle subscriber data and updates self."""
		#rospy.loginfo(rospy.get_name() + " I got data %s", data)
		
		#sets the position data	
		x = data.transform.translation.x #hi
		y = data.transform.translation.y
		z = data.transform.translation.z
		
		#converts the quaternion to euler angles
		euler = self.quaternion_to_euler(data.transform.rotation)
		
		#sets the orientation data	
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		
		#grabs the current time stamp
		current_time = data.header.stamp.secs + data.header.stamp.nsecs/1E9

		#set states
		self.past_state = self.current_state
		self.current_state = Robot_State(x,y,z,roll,pitch,yaw,current_time)

		#calls the trajectory tracker
		if self.past_state != None:
			v, theta = self.trajectory_tracking(0.5, 0)
			[v_average, theta_average] = self.moving_average(v,theta)
			self.send_twist_message(v_average,theta_average)

	def quaternion_to_euler(self, quaternion):
		"""converts a quaternion to euler angles"""
		#rospy.loginfo("quat to euler")
		#type(pose) = geometry_msgs.msg.Pose
		euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
		return euler

	def on_tf(self, data):
		transform = None
		for d in data:
			if d.header.frame_id == "odom":
				transform = d.transform
		if transform != None:
			print transform
			t = TransformStamped()
			t.transform = transform
			self.pub.publish(t)

# Main function.
if __name__ == '__main__':
	# Initialize the node and name it.
	print "Initiating server node..."
	rospy.init_node('PID_Controller')
    
	try:
		controller_server = Controller()

	except rospy.ROSInterruptException:
		rospy.logerror("Failed to start server node.")
		pass
