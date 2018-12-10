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

def processor(bag_file='test.bag', x_data_range=2):

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
	time_base = -1
	startdada = 9E99
	enddada = 9E99

	if (vicon_msgs):
		nodada = True

		count = 0

		for topic, msg, t0 in bag.read_messages(topics=vicon_topic):

			if msg.transform.translation.x >= -x_data_range and nodada:
				time_base = msg.header.stamp.to_sec()
				startdada = (msg.header.stamp.to_sec() - time_base)
				nodada = False

			if nodada == False:
				count += 1

			if msg.transform.translation.x > x_data_range:
				enddada = (msg.header.stamp.to_sec() - time_base)
				break

		vicon_time = [None for x in xrange(count)]
		vicon_px    = [None for x in xrange(count)]
		vicon_py    = [None for x in xrange(count)]
		vicon_pz    = [None for x in xrange(count)]
		vicon_rx    = [None for x in xrange(count)]
		vicon_ry    = [None for x in xrange(count)]
		vicon_rz    = [None for x in xrange(count)]

		pt = 0

		for topic, msg, t0 in bag.read_messages(topics=vicon_topic):

			if (msg.header.stamp.to_sec() - time_base) >= startdada and (msg.header.stamp.to_sec() - time_base) <= enddada:
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
		count = 0

		for topic, msg, t0 in bag.read_messages(topics=cmd_vel_topic):
			if (msg.header.stamp.to_sec() - time_base) >= startdada and (msg.header.stamp.to_sec() - time_base) <= enddada:
				count += 1

		cmd_time = [0 for x in xrange(count)]
		cmd_vx = [0 for x in xrange(count)]
		cmd_wz = [0 for x in xrange(count)]
		# cmd vel

		pt = 0

		for topic, msg, t0 in bag.read_messages(topics=cmd_vel_topic):
			if (msg.header.stamp.to_sec() - time_base) >= startdada and (msg.header.stamp.to_sec() - time_base) <= enddada:
				cmd_time[pt] = (msg.header.stamp.to_sec() - time_base)
				cmd_vx[pt]   = msg.twist.linear.x
				cmd_vx[pt]   = msg.twist.linear.x
				cmd_wz[pt]   = msg.twist.angular.z*180.0/pi

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


	v_error_rms = np.sqrt(np.mean(np.array(veloc)**2))
	theta_error_rms = np.sqrt(np.mean(np.array(vicon_rz)**2))
	print "v_error_rms,", v_error_rms
	print "theta_error_rms,", theta_error_rms

	v_temp = [abs(x) for x in veloc]
	theta_temp = [abs(x) for x in vicon_rz]

	v_error_max = max(v_temp)
	theta_error_max = max(theta_temp)

	print "v_error_max,", v_error_max
	print "theta_error_max,", theta_error_max

	v_error_avg = np.mean(np.array(veloc))
	theta_error_avg = np.mean(np.array(vicon_rz))

	file = open("outdata.txt","a") 
	data_str = "%s, %s, %s, %s, %s, %s, %s,\n" % (bag_file, v_error_rms, theta_error_rms, v_error_max, theta_error_max, v_error_avg, theta_error_avg)
	file.write(data_str)
	file.close() 

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

	name = "%s Controller with %s Disturbance, Trial #%s: " % (cntrllr, disturbance, trial)
	
	grid_color = 'grey'
	plot_lw = 1.5

	#print len(pd.cmd_t), len(pd.vicon_t)
	tmin, tmax = min_max([pd.vicon_t, pd.cmd_t])
	#print "  tmin=",str(tmin),"  tmax=",str(tmax)

	#print "  Plot commands ..."
	pwmin, pwmax = min_max([pd.cmd_vx, pd.cmd_wz])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	a = subplt.plot(pd.cmd_t,pd.cmd_vx,'r', linewidth=plot_lw)
	subplt.set_ylabel('linear velocity (m/s)')
	subplt.set_xlabel('time (s)')
	#subplt.legend(['commanded linear velocity (m/s)'])

	subplt2 = subplt.twinx()	

	b = subplt2.plot(pd.cmd_t,pd.cmd_wz,'b', linestyle='-', linewidth=plot_lw)
	subplt2.legend(a+b,['commanded linear velocity (m/s)','commanded angular velocity (deg/s)'], fancybox=True, framealpha=0.5)
	subplt2.axis([tmin, tmax, pwmin, pwmax])
	subplt2.set_ylabel('angular velocity (deg/s)')
	fig.suptitle(name + "Commands")
	fig.savefig(saver + " Commands.png")
	plt.close()

	#print "  Plot vicon ..."

	pmin, pmax = min_max([pd.vicon_px, pd.vicon_py])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	subplt.plot(pd.vicon_t,pd.vicon_px,'r', linewidth=plot_lw)
	subplt.plot(pd.vicon_t,pd.vicon_py,'g', linestyle='-', linewidth=plot_lw)
	# subplt.plot(vicon_time,vicon_pz,'b')
	subplt.legend(['global position x (m)', 'global position y (m)'], fancybox=True, framealpha=0.5)
	subplt.axis([tmin, tmax, pmin, pmax])

	subplt.set_ylabel('position (m)')
	subplt.set_xlabel('time (s)')
	fig.suptitle(name + "Vicon Data")
	fig.savefig(saver + " Vicon Data.png")
	plt.close()


	rmin, rmax = min_max([pd.vicon_rz])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	# subplt.plot(vicon_time,vicon_rx,'r')
	# subplt.plot(vicon_time,vicon_ry,'g')
	subplt.plot(pd.vicon_t,pd.vicon_rz,'b', linewidth=plot_lw)
	subplt.legend(['theta (yaw) error (deg)'], fancybox=True, framealpha=0.5)
	subplt.axis([tmin, tmax, rmin, rmax])

	subplt.set_ylabel('angle (deg)')
	subplt.set_xlabel('time (s)')
	fig.suptitle(name + "Angle Error")
	fig.savefig(saver + " Angle Error.png")
	plt.close()


	#print "  Plot velocity ..."

	vmin, vmax = min_max([pd.veloc])
	vtmin, vtmax = min_max([pd.veloc_t])

	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	subplt.plot(pd.veloc_t,pd.veloc,'r', linewidth=plot_lw)
	# subplt.plot(vicon_time,vicon_pz,'b')
	subplt.legend(['linear velocity error (m/s)'], fancybox=True, framealpha=0.5)
	subplt.axis([vtmin, vtmax, vmin, vmax])
	subplt.set_ylabel('linear velocity (m/s)')
	subplt.set_xlabel('time (s)')
	fig.suptitle(name + "Velocity Error")
	fig.savefig(saver + " Velocity Error.png")
	plt.close()

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
	file = open("outdata.txt","w")
	file.write("Filename, v_error_rms, theta_error_rms, v_error_max, theta_error_max, v_error_avg, theta_error_avg,\n")
	file.close() 

	for bag_file in glob.glob("PID_board_1.bag"):
		if "fixed" in bag_file:
			continue
		print(bag_file)
		a = bag_file[:-4].split('_')
		cntrllr = a[0].upper()
		disturbance = a[1].title()
		trial = a[2].title()

		repaired_file = fix_data(bag_file)

		pd = processor(repaired_file, 1.5)
		plotter(pd, cntrllr, disturbance, trial, bag_file[:-4])

