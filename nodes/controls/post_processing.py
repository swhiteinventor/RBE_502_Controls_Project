#!/usr/bin/python
import yaml
import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np
import tf.transformations
from math import pi
import glob, os
from geometry_msgs.msg import TwistStamped

class Post_Data():
	def __init__(self):
		return None

def processor(bag_file='test.bag'):

	start_time = 0
	end_time   = 9E9

	#print "Read bag file ..."
	bag = rosbag.Bag(bag_file, 'r')

	#print "Get topic info..."
	info_dict = yaml.load(bag._get_yaml_info())
	#print(info_dict['topics'])

	#print "Get list ..."
	topic_list = []
	cmd_msgs = 0
	state_msgs = 0

	# ROS Topics
	vicon_msgs		= []
	cmd_vel_msgs	= []

	# Vicon Data
	vicon_topic = '/vicon/turtlebot_traj_track/turtlebot_traj_track'
	vicon_time	= []

	vicon_px	= []
	vicon_py	= []
	vicon_pz	= []

	vicon_ox	= []
	vicon_oy	= []
	vicon_oz	= []

	# CMD Data
	cmd_vel_topic = '/mobile_base/commands/velocity_FIXED'
	cmd_time	= []

	cmd_vx 		= []
	cmd_wz		= []

	for topic_info in info_dict['topics']:
		topic = topic_info['topic']
		topic_list.append(topic)

		# if topic == clock_topic:
		# 	clock_msgs = topic_info['messages']
		if topic == vicon_topic:
			vicon_msgs = topic_info['messages']
		if topic == cmd_vel_topic:
			cmd_vel_msgs = topic_info['messages']

	#print topic_list

	#print "Message counts:"
	# print "  clock msgs="+str(  clock_msgs	)
	#print "  vicon msgs="+str(  vicon_msgs	)
	#print "  cmd_vel msgs="+str(  cmd_vel_msgs	)

	#print "Process messages ..."
	time_base = -1;

	if (vicon_msgs):
		vicon_time = [0 for x in xrange(vicon_msgs)]
		vicon_px    = [0 for x in xrange(vicon_msgs)]
		vicon_py    = [0 for x in xrange(vicon_msgs)]
		vicon_pz    = [0 for x in xrange(vicon_msgs)]
		vicon_rx    = [0 for x in xrange(vicon_msgs)]
		vicon_ry    = [0 for x in xrange(vicon_msgs)]
		vicon_rz    = [0 for x in xrange(vicon_msgs)]

		pt = 0

		for topic, msg, t0 in bag.read_messages(topics=vicon_topic):
				if (time_base < 0):
					time_base = msg.header.stamp.to_sec()
					#print "time base =",time_base

				vicon_time[pt] = (msg.header.stamp.to_sec() - time_base)
				vicon_px[pt]    = msg.transform.translation.x
				vicon_py[pt]    = msg.transform.translation.y
				vicon_pz[pt]    = msg.transform.translation.z

				q = msg.transform.rotation
				e = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
				vicon_rx[pt]    = e[0]*180.0/pi
				vicon_ry[pt]    = e[1]*180.0/pi
				vicon_rz[pt]    = e[2]*180.0/pi

				pt = pt + 1

		start_time = min(vicon_time)
		end_time = max(vicon_time)

		#print "Clock start time ="+str(start_time)
		#print "Set end time ="+str(end_time)

		#print "vicon start time ="+str(min(vicon_time))
		#print "vicon end time ="+str(max(vicon_time))

	if (cmd_vel_msgs):
		cmd_time = [0 for x in xrange(cmd_vel_msgs)]
		cmd_vx = [0 for x in xrange(cmd_vel_msgs)]
		cmd_wz = [0 for x in xrange(cmd_vel_msgs)]
		# cmd vel

		pt = 0

		for topic, msg, t0 in bag.read_messages(topics=cmd_vel_topic):
			cmd_time[pt] = (msg.header.stamp.to_sec() - time_base)
			cmd_vx[pt]   = msg.twist.linear.x
			cmd_vx[pt]   = msg.twist.linear.x
			cmd_wz[pt]   = msg.twist.angular.z

			pt = pt + 1

		#print "Cmd start time ="+str(min(cmd_time))
		#print "Cmd end time ="+str(max(cmd_time))

	#print "Close bag!"
	bag.close()


	veloc_t = []
	veloc = []
	for i in range(1, len(vicon_time)):
		delta_x = vicon_px[i]-vicon_px[i-1]
		delta_y = vicon_py[i]-vicon_py[i-1]
		delta_d = (delta_x**2 + delta_y**2)**0.5
		delta_t = vicon_time[i]-vicon_time[i-1]

		veloc.append(delta_d/delta_t - 0.325)
		veloc_t.append(vicon_time[i])


	pd = Post_Data()
	pd.vicon_t = vicon_time
	pd.vicon_px = vicon_px
	pd.vicon_py = vicon_py
	pd.vicon_rz = vicon_rz
	pd.veloc = veloc
	pd.veloc_t = veloc_t

	pd.cmd_vx = cmd_vx
	pd.cmd_wz = cmd_wz
	pd.cmd_t = cmd_time

	return pd

def min_max(array):
	a = []
	b = []
	for x in array:
		a.append(min(x))
		b.append(max(x))
	mi = min(a) - 0.2
	ma = max(b) + 0.2

	return mi, ma

def plotter(pd, cntrllr="PID", disturbance="board", trial="1", saver=""):

	name = "%s controller with %s disturbance. Trial #%s: " % (cntrllr, disturbance, trial)
	

	#print len(pd.cmd_t), len(pd.vicon_t)
	tmin, tmax = min_max([pd.vicon_t, pd.cmd_t])
	#print "  tmin=",str(tmin),"  tmax=",str(tmax)

	#print "  Plot commands ..."
	pwmin, pwmax = min_max([pd.cmd_vx, pd.cmd_wz])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.hold(True)
	subplt.plot(pd.cmd_t,pd.cmd_vx,'r')
	subplt.plot(pd.cmd_t,pd.cmd_wz,'g')
	subplt.axis([tmin, tmax, pwmin, pwmax])

	subplt.set_ylabel('m/s rad/s')
	subplt.set_xlabel('time')
	subplt.legend(['vx_cmd (m/s)', 'wz_cmd (rad/s)'])
	fig.suptitle(name + "Commands")
	fig.savefig(saver + " Commands.png")

	#print "  Plot vicon ..."

	pmin, pmax = min_max([pd.vicon_px, pd.vicon_py])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.hold(True)
	subplt.plot(pd.vicon_t,pd.vicon_px,'r')
	subplt.plot(pd.vicon_t,pd.vicon_py,'g')
	# subplt.plot(vicon_time,vicon_pz,'b')
	subplt.axis([tmin, tmax, pmin, pmax])

	subplt.set_ylabel('meters')
	subplt.set_xlabel('time')
	subplt.legend(['px_actual (m)', 'py_actual (m)'])
	fig.suptitle(name + "Vicon Data")
	fig.savefig(saver + " Vicon Data.png")


	rmin, rmax = min_max([pd.vicon_rz])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.hold(True)
	# subplt.plot(vicon_time,vicon_rx,'r')
	# subplt.plot(vicon_time,vicon_ry,'g')
	subplt.plot(pd.vicon_t,pd.vicon_rz,'b')
	subplt.axis([tmin, tmax, rmin, rmax])

	subplt.set_ylabel('degrees')
	subplt.set_xlabel('time')
	subplt.legend(['rz_error (degrees)'])
	fig.suptitle(name + "Angle Error")
	fig.savefig(saver + " Angle Error.png")


	#print "  Plot velocity ..."

	vmin, vmax = min_max([pd.veloc])
	vtmin, vtmax = min_max([pd.veloc_t])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color='black', linestyle='-', linewidth=1)
	subplt.hold(True)
	subplt.plot(pd.veloc_t,pd.veloc,'b')
	# subplt.plot(vicon_time,vicon_pz,'b')
	subplt.axis([vtmin, vtmax, vmin, vmax])
	subplt.set_ylabel('meters/second')
	subplt.set_xlabel('time')
	subplt.legend(['velocity_error (m/s)'])
	fig.suptitle(name + "Velocity Error")
	fig.savefig(saver + " Velocity Error.png")

def fix_data(bag_file):

    repaired_file = bag_file.replace(".bag", "_fixed.bag")  # Output bag path

    topic1 = '/mobile_base/commands/velocity'  # Target topic you want to repair
    topic2 = '/vicon/turtlebot_traj_track/turtlebot_traj_track'
    
    # Open bag
    bag = rosbag.Bag(bag_file, 'r')
    fix_bag = rosbag.Bag(repaired_file, "w")  # Create a repaired bag

    # Iterate through bag
    for topic, msg, t in bag.read_messages():
        # Add time back to the target message header
        if topic == topic1:
            m = TwistStamped()
            m.twist = msg
            m.header.stamp.secs = t.secs
            m.header.stamp.nsecs = t.nsecs

            # Write message to bag
            fix_bag.write(topic+"_FIXED", m, t)

        if topic == topic2:

            # Write message to bag
            fix_bag.write(topic2, msg, t)

    # Close bag - Very important else you'll have to reindex it
    fix_bag.close()
    return repaired_file

if __name__ == '__main__':
	os.chdir(".")
	for bag_file in glob.glob("NLF_board_2.bag"):
		print(bag_file)
		a = bag_file[:-4].split('_')
		cntrllr = a[0]
		disturbance = a[1]
		trial = a[2]

		repaired_file = fix_data(bag_file)

		pd = processor(repaired_file)
		plotter(pd, cntrllr, disturbance, trial, bag_file[:-4])

