#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example Python node to listen & publish on a specific topic."""

# Import required Python code.
import roslib; roslib.load_manifest('controls')
import rospy
import Queue

from std_msgs.msg import Empty, String


class PID_Controller():

    def __init__(self):
        rospy.loginfo("Server node started.")

        rospy.Subscriber('topic/name/here', Empty, self.on_data)

        self.pub = rospy.Publisher('topic/name/here', String, latch=True, queue_size=1)
        s = String()
        s.String = ""
        self.pub.publish(s)
        rospy.spin()


    def on_data(self, data):
            """Handle subscriber data."""
            rospy.loginfo(rospy.get_name() + " I got data %s", data)

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