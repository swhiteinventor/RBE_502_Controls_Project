#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import Queue

from Robot_State import Robot_State
from std_msgs.msg import Empty, String
import tf.transformations
from geometry_msgs.msg import Twist, TransformStamped
from PID_Controller import PID_controller

class Controller():

	def __init__(self):
		"""initializes the controls node"""
		rospy.loginfo("Server node started.")
	
		#initializes the robot's position and orientation and time
		self.past_state = None
		self.current_state = None

		#initializes gains
		self.kp_v = 10
		self.ki_v = 0.2
		self.kd_v = 5
		self.kp_theta = 10
		self.ki_theta = 0.2
		self.kd_theta = 5

		self.controller = "PID"
	
		#change wand to turtlebot later
		rospy.Subscriber('/vicon/turtlebot/turtlebot', TransformStamped, self.on_data) 
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, latch=True, queue_size=1)

		rospy.spin()

	def trajectory_tracking(self, desired_v, desired_theta):
		#rospy.loginfo("traj tracking")

		state_dot = self.calculate_derivatives()
		current_v = ((state_dot.x)**2+(state_dot.y)**2)**0.5 #do we need z in here?
		error_v = self.calculate_error(current_v, desired_v)
		current_theta = self.current_state.yaw
		error_theta = self.calculate_error(current_theta, desired_theta)
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

	def send_twist_message(self, v, theta):
		"""takes in the velocity and theta"""
		v = min(v,3)
		v = max(v,-3)
		theta = min(theta,3)
		theta = max(theta,-3)
		rospy.loginfo("linear x: %.4f angular z: %.4f" % (v, theta))
		t = Twist()
		t.linear.x = v
		t.linear.y = 0
		t.linear.z = 0
	 	t.angular.x = 0
		t.angular.y = 0
		t.angular.z = theta
		self.pub.publish(t)


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

		if self.past_state != None:
			v, theta = self.trajectory_tracking(0.5, 0)
			self.send_twist_message(v,theta)

	def quaternion_to_euler(self, quaternion):
		"""converts a quaternion to euler angles"""
		#rospy.loginfo("quat to euler")
		#type(pose) = geometry_msgs.msg.Pose
		euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
		return euler

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
