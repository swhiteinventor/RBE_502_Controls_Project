#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import Queue

from std_msgs.msg import Empty, String
import tf.transformations
from geometry_msgs.msg import Twist, TransformStamped

class PID_Controller():

	def __init__(self):
		"""initializes the controls node"""
		rospy.loginfo("Server node started.")
	
		#initializes the robot's position and orientation
		self.x = 0
		self.y = 0
		self.z = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
	
		#change wand to turtlebot later
		rospy.Subscriber('/vicon/wand/wand', TransformStamped, self.on_data) 
		self.pub = rospy.Publisher('topic/name/here', Twist, latch=True, queue_size=1)

	def trajectory_tracking():
		hi=1

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
		self.x = data.linear.x
		self.y = data.linear.y
		self.z = data.linear.z
		#converts the quaternion to euler angles
		euler = quaternion_to_euler(data.angular)
		#sets the orientation data	
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]

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
