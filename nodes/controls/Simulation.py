#!/usr/bin/env python
# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates

class Simulation():

	def __init__(self):
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.on_tf) 
		self.pub_tf = rospy.Publisher('/vicon/turtlebot_traj_track/turtlebot_traj_track', TransformStamped, latch=True, queue_size=1)

		rospy.spin()

	def on_tf(self, data):

		pose = data.pose[1]

		h = Header()
		h.stamp = rospy.Time.now()

		t = TransformStamped()
		t.transform.translation = pose.position
		t.transform.rotation = pose.orientation
		t.header = h

		print "Published Data:\n", t
		self.pub_tf.publish(t)

# Main function.
if __name__ == '__main__':
	# Initialize the node and name it.
	print "Initiating simulation node..."
	rospy.init_node('Simulation')
    
	try:
		Simulation_server = Simulation()

	except rospy.ROSInterruptException:
		rospy.logerror("Failed to start server node.")
		pass
