#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import Queue

from RobotState import Robot_State
from std_msgs.msg import Empty, String
import tf.transformations
from geometry_msgs.msg import Twist, TransformStamped
from PID_Controller import PID_Control

class Controller():

	def __init__(self):
		"""initializes the controls node"""
		rospy.loginfo("Server node started.")
	
		#initializes the robot's position and orientation and time
		self.past_state = Robot_State()
		self.current_state = Robot_State()

		#initializes gains
		self.kp_v = 10
		self.ki_v = 0.2
		self.kd_v = 5
		self.kp_theta = 10
		self.ki_theta = 0.2
		self.kd_theta = 5

		self.controller = "PID"
	
		#change wand to turtlebot later
		rospy.Subscriber('/vicon/wand/wand', TransformStamped, self.on_data) 
		self.pub = rospy.Publisher('topic/name/here', Twist, latch=True, queue_size=1)


	def trajectory_tracking(self, desired_v, desired_theta):

		state_dot = self.calculate_derivatives()
		current_v = ((state_dot.x)^2+(state_dot.y)^2)^0.5 #do we need z in here?
		error_v = self.calculate_error()
		return (v, theta)

	def calculate_derivatives(self):
		delta_state = self.current_state - self.past_state
		#calculates derivatives, where time (t) is the time step
		state_dot = delta_state.divide_by_time()
		return state_dot

	def calculate_error(self, current, goal):
		error = current - goal
		return error

	def send_twist_message(self, v, theta):
		"""takes in the velocity and theta"""
		t = Twist()
		t.linear.x = v
		t.linear.y = 0
		t.linear.z = 0
	 	t.angular.x = 0
		t.angular.y = 0
		t.angular.z = theta
		self.pub.publish(t)
		rospy.spin()


	def on_data(self, data):
		
		"""Callback function that handle subscriber data and updates self."""
		rospy.loginfo(rospy.get_name() + " I got data %s", data)
		
		#sets the position data	
		x = data.linear.x #hi
		y = data.linear.y
		z = data.linear.z
		
		#converts the quaternion to euler angles
		euler = quaternion_to_euler(data.angular)
		
		#sets the orientation data	
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		
		#grabs the current time stamp
		current_time = data.header.stamp.sec + data.header.stamp.nsec/1E9

		#set states
		self.past_state = self.current_state
		self.current_state = Robot_State(x,y,z,roll,pitch,yaw,current_time)

		if self.controller == "PID":
			PID_Control(self)

	def quaternion_to_euler(quaternion):
		"""converts a quaternion to euler angles"""
		#type(pose) = geometry_msgs.msg.Pose
		euler = tf.transformations.euler_from_quaternion(quaternion)
		return euler

# Main function.
if __name__ == '__main__':
	# Initialize the node and name it.
	print "Initiating server node..."
	rospy.init_node('PID_Controller')
    
	try:
		rs = PID_Controller()

	except rospy.ROSInterruptException:
		rospy.logerror("Failed to start server node.")
		pass