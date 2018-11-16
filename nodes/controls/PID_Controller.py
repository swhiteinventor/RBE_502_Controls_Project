#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import Queue

from RobotState import RobotState
from std_msgs.msg import Empty, String
import tf.transformations
from geometry_msgs.msg import Twist, TransformStamped

class PID_Controller():

	def __init__(self):
		"""initializes the controls node"""
		rospy.loginfo("Server node started.")
	
		#initializes the robot's position and orientation and time
		self.past_state = RobotState()
		self.current_state = RobotState()

		#initializes gains
		self.kp_v = 10
		self.ki_v = 0.2
		self.kd_v = 5
		self.kp_theta = 10
		self.ki_theta = 0.2
		self.kd_theta = 5
	
		#change wand to turtlebot later
		rospy.Subscriber('/vicon/wand/wand', TransformStamped, self.on_data) 
		self.pub = rospy.Publisher('topic/name/here', Twist, latch=True, queue_size=1)


	def trajectory_tracking(self, desired_v, desired_theta):
		state_dot = current_state - past_state
		#calculates derivatives
		x_dot = state_dot.x/state_dot.t
		y_dot = state_dot.y/state_dot.t
		yaw_dot = state_dot.yaw/state_dot.t

		current_v = (x_dot)^2+(y_dot)^2)^0.5
		error_v = calculate_error(
		return v, theta

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
		self.x = data.linear.x #hi
		self.y = data.linear.y
		self.z = data.linear.z
		#converts the quaternion to euler angles
		euler = quaternion_to_euler(data.angular)
		#sets the orientation data	
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]
		#grabs the current time stamp
		self.previous_time = self.current_time		
		self.current_time = data.header.stamp.sec + data.header.stamp.nsec/1E9

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