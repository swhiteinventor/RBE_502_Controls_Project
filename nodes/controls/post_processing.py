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

"""
This file post processes bag files within the current directory and outputs a .csv file along with .png graphs.
Notes:
	current directory must be writable.
	.bag files are expected in a specific naming format.
	vicon object shall be called turtlebot_traj_track

To run this file:
	navigate to the folder with bag files then...
	rosrun controls post_processing.py

Bag File Format:
	controller_disturbance_trialNum.bag
	PID_board_1.bag
"""

class Post_Data():
	"""
	Data wrapper for transferring processed data back and passing to the plotter
	"""

	def __init__(self):
		"""
		Initiallize Post_Data with no values

		Parameters:
			None
		Returns:
			None
		"""
		return None

def processor(bag_file='test.bag', x_data_range=2, v_desired=0.325, theta_desired=0, DEBUG=False):
	"""
	Process the bag file to pull out relevent data

	Parameters:
		bag_file (opt) - the file to process data from
		x_data_range (opt) - range greater than +/- to ignore data beyond
		v_desired (opt) - desired velocity for velocity error
		theta_desired (opt) - desired theta in degrees for theta error
		DEBUG (opt) - debug print statements
	Returns:
		Post_Data - with relavent data arrays to be plotted
	"""
		
	start_time = 0
	end_time   = 9E9

	if DEBUG:
		print "Read bag file ..."
	bag = rosbag.Bag(bag_file, 'r')

	info_dict = yaml.load(bag._get_yaml_info())

	topic_list = []

	cmd_msgs = 0
	state_msgs = 0

	# Setup Message Arrays

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

		if topic == vicon_topic:
			vicon_msgs = topic_info['messages']
		if topic == cmd_vel_topic:
			cmd_vel_msgs = topic_info['messages']

	if DEBUG:
		print topic_list

	if DEBUG:
		print "Message counts:"
		print "  vicon msgs="+str(  vicon_msgs	)
		print "  cmd_vel msgs="+str(  cmd_vel_msgs	)

		print "Process messages ..."

	# Time in seconds to subtract from ROS Time
	time_base = -1
	# Start time of data to keep
	startdada = 9E99
	# Stop time of data to keep
	enddada = 9E99

	# if vicon message count is not None
	if (vicon_msgs):
		nodada = True

		count = 0

		# for message published to vicon topic
		for topic, msg, t0 in bag.read_messages(topics=vicon_topic):

			# if the x value is in range of data to keep and haven't kept data yet
			if msg.transform.translation.x >= -x_data_range and nodada:

				# get the current time
				time_base = msg.header.stamp.to_sec()

				# set the start of data to be 0
				startdada = (msg.header.stamp.to_sec() - time_base)

				# mark data as being kept
				nodada = False

			# count the number of data points being kept
			if nodada == False:
				count += 1

			# if x is beyond uppder data range
			if msg.transform.translation.x > x_data_range:

				# mark the end time for data
				enddada = (msg.header.stamp.to_sec() - time_base)

				# stop iterating through the loop
				break

		# setup arrays with the current data point count
		vicon_time = [None for x in xrange(count)]
		vicon_px    = [None for x in xrange(count)]
		vicon_py    = [None for x in xrange(count)]
		vicon_pz    = [None for x in xrange(count)]
		vicon_rx    = [None for x in xrange(count)]
		vicon_ry    = [None for x in xrange(count)]
		vicon_rz    = [None for x in xrange(count)]

		pt = 0

		# now iterate through each message again
		for topic, msg, t0 in bag.read_messages(topics=vicon_topic):

			# if data is in time frame to be kept (derived above)
			if (msg.header.stamp.to_sec() - time_base) >= startdada and (msg.header.stamp.to_sec() - time_base) <= enddada:

				# save the x,y,z data point
				vicon_time[pt] = (msg.header.stamp.to_sec() - time_base)
				vicon_px[pt]    = msg.transform.translation.x
				vicon_py[pt]    = msg.transform.translation.y
				vicon_pz[pt]    = msg.transform.translation.z

				# save the rotation data as a euler in degrees
				q = msg.transform.rotation
				e = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
				vicon_rx[pt]    = e[0]*180.0/pi
				vicon_ry[pt]    = e[1]*180.0/pi
				vicon_rz[pt]    = e[2]*180.0/pi - theta_desired

				pt = pt + 1

		# set the start and end times based on vicon data
		start_time = min(vicon_time)
		end_time = max(vicon_time)

		if DEBUG:
			print "Clock start time ="+str(start_time)
			print "Set end time ="+str(end_time)

			print "vicon start time ="+str(min(vicon_time))
			print "vicon end time ="+str(max(vicon_time))

	# if cmd_vel messages is not none
	if (cmd_vel_msgs):
		count = 0

		# iterate through each cmd vel message
		for topic, msg, t0 in bag.read_messages(topics=cmd_vel_topic):

			# get the count of messages within the time range determined above
			if (msg.header.stamp.to_sec() - time_base) >= startdada and (msg.header.stamp.to_sec() - time_base) <= enddada:
				count += 1

		# generate arrays of length count
		cmd_time = [0 for x in xrange(count)]
		cmd_vx = [0 for x in xrange(count)]
		cmd_wz = [0 for x in xrange(count)]

		pt = 0

		# iterate through each message again
		for topic, msg, t0 in bag.read_messages(topics=cmd_vel_topic):

			# if message within time range
			if (msg.header.stamp.to_sec() - time_base) >= startdada and (msg.header.stamp.to_sec() - time_base) <= enddada:

				# save the data point
				cmd_time[pt] = (msg.header.stamp.to_sec() - time_base)
				cmd_vx[pt]   = msg.twist.linear.x
				cmd_vx[pt]   = msg.twist.linear.x
				cmd_wz[pt]   = msg.twist.angular.z*180.0/pi

				pt = pt + 1

		if DEBUG:
			print "Cmd start time ="+str(min(cmd_time))
			print "Cmd end time ="+str(max(cmd_time))

	if DEBUG:
		print "Close bag!"
	bag.close()

	# process velocity data from vicon position data
	veloc_t = []
	veloc = []

	# for msg in len(vicon time) but start at 1
	for i in range(1, len(vicon_time)):
		# calculate delta x, y, net(distance), and t
		delta_x = vicon_px[i]-vicon_px[i-1]
		delta_y = vicon_py[i]-vicon_py[i-1]
		delta_d = (delta_x**2 + delta_y**2)**0.5
		delta_t = vicon_time[i]-vicon_time[i-1]

		# calculate velocity error
		veloc.append(delta_d/delta_t - v_desired)
		veloc_t.append(vicon_time[i])

	# calculate velocity and theta error RMS
	v_error_rms = np.sqrt(np.mean(np.array(veloc)**2))
	theta_error_rms = np.sqrt(np.mean(np.array(vicon_rz)**2))

	if DEBUG:
		print "v_error_rms,", v_error_rms
		print "theta_error_rms,", theta_error_rms

	# get abs value of each value in veloc and rotation error arrays
	v_temp = [abs(x) for x in veloc]
	theta_temp = [abs(x) for x in vicon_rz]

	# get the max deviation
	v_error_max = max(v_temp)
	theta_error_max = max(theta_temp)

	if DEBUG:
		print "v_error_max,", v_error_max
		print "theta_error_max,", theta_error_max

	# get the simple average of velocity and theta erros
	v_error_avg = np.mean(np.array(veloc))
	theta_error_avg = np.mean(np.array(vicon_rz))

	# save the data to outdata.csv
	file = open("outdata.csv","a") 
	data_str = "%s, %s, %s, %s, %s, %s, %s,\n" % (bag_file, v_error_rms, theta_error_rms, v_error_max, theta_error_max, v_error_avg, theta_error_avg)
	file.write(data_str)
	file.close() 

	# put calculated data into post data wrapper for transport
	pd = Post_Data()
	pd.vicon_t = vicon_time
	pd.vicon_px = vicon_px
	pd.vicon_py = vicon_py
	pd.vicon_pz = vicon_pz
	pd.vicon_rz = vicon_rz
	pd.veloc = veloc
	pd.veloc_t = veloc_t

	pd.cmd_vx = cmd_vx
	pd.cmd_wz = cmd_wz
	pd.cmd_t = cmd_time

	return pd

def min_max(array):
	"""
	get the min and max of multiple datasets (arrays of arrays) with buffer range

	Parameters:
		array - array containing message arrays
	Returns:
		min - min value from all the arrays - 0.2
		max - max value from all the arrays + 0.2
	"""
	a = []
	b = []

	# for array x in the main array
	for x in array:
		# get the min and max of that array
		a.append(min(x))
		b.append(max(x))

	# get the most significant min and max and add a buffer
	mi = min(a) - 0.2
	ma = max(b) + 0.2

	return mi, ma

def plotter(pd, cntrllr="____", disturbance="_____", trial="__", saver="", DEBUG=False):
	"""
	Plots the processed data and exports to .png files

	Parameters:
		pd - processed data
		cntrllr (opt) - controller name
		disturbance (opt) - type of disturbance
		trial (opt) - number of the trial
		saver (opt) - name prefix for the saved files
		DEBUG (opt) - Debug print messages
	Returns:
		None
	"""

	name = "%s Controller with %s Disturbance, Trial #%s: " % (cntrllr, disturbance, trial)

	if DEBUG:
		print name
	
	grid_color = 'grey'
	plot_lw = 1.5

	# time min and time max for plots
	tmin, tmax = min_max([pd.vicon_t, pd.cmd_t])

	if DEBUG:
		print "  Plot commands ..."

	# velocity, omega min and max for command plot
	pwmin, pwmax = min_max([pd.cmd_vx, pd.cmd_wz])

	# configure the plot
	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	# plot the cmd_veloc over time in red
	a = subplt.plot(pd.cmd_t,pd.cmd_vx,'r', linewidth=plot_lw)
	# setup scale # 1
	subplt.set_ylabel('linear velocity (m/s)')
	subplt.set_xlabel('time (s)')
	subplt2 = subplt.twinx()
	# plot the cmd_omega over time in blue
	b = subplt2.plot(pd.cmd_t,pd.cmd_wz,'b', linestyle='-', linewidth=plot_lw)
	# setup the legend
	subplt2.legend(a+b,['commanded linear velocity (m/s)','commanded angular velocity (deg/s)'], fancybox=True, framealpha=0.5)
	# setup scale # 2
	subplt2.axis([tmin, tmax, pwmin, pwmax])
	subplt2.set_ylabel('angular velocity (deg/s)')
	# name and save the plot
	fig.suptitle(name + "Commands")
	fig.savefig(saver + " Commands.png")
	plt.close()

	if DEBUG:
		print "  Plot vicon ..."

	# positional min and max values for vicon plot
	pmin, pmax = min_max([pd.vicon_px, pd.vicon_py])

	# configure the plot
	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	# plot position_x over time in red
	subplt.plot(pd.vicon_t,pd.vicon_px,'r', linewidth=plot_lw)
	# plot position_y over time in green
	subplt.plot(pd.vicon_t,pd.vicon_py,'g', linestyle='-', linewidth=plot_lw)
	# setup the legend
	subplt.legend(['global position x (m)', 'global position y (m)'], fancybox=True, framealpha=0.5)
	subplt.axis([tmin, tmax, pmin, pmax])
	subplt.set_ylabel('position (m)')
	subplt.set_xlabel('time (s)')
	# name and save the plot
	fig.suptitle(name + "Vicon Data")
	fig.savefig(saver + " Vicon Data.png")
	plt.close()

	# rotation min and max for vicon plot # 2
	rmin, rmax = min_max([pd.vicon_rz])

	# configure the plot
	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	# plot rotational_z over time in blue
	subplt.plot(pd.vicon_t,pd.vicon_rz,'b', linewidth=plot_lw)
	# setup legend
	subplt.legend(['theta (yaw) error (deg)'], fancybox=True, framealpha=0.5)
	subplt.axis([tmin, tmax, rmin, rmax])
	subplt.set_ylabel('angle (deg)')
	subplt.set_xlabel('time (s)')
	# name and save the plot
	fig.suptitle(name + "Angle Error")
	fig.savefig(saver + " Angle Error.png")
	plt.close()

	if DEBUG:
		print "  Plot velocity ..."

	# setup the velocity min and max
	vmin, vmax = min_max([pd.veloc])
	# setup the time min and max for velocity
	vtmin, vtmax = min_max([pd.veloc_t])

	# temp name change
	list_y = pd.veloc
	list_x = pd.veloc_t

	# configure the plot
	fig = plt.figure()
	subplt = fig.add_subplot(111)
	subplt.grid(color=grid_color, linestyle='-', linewidth=.5)
	subplt.hold(True)
	# setup best fit poly of velocity data
	poly = np.polyfit(list_x,list_y,15)
	poly_y = np.poly1d(poly)(list_x)
	# plot the raw velocity over time in red
	plt.plot(list_x,list_y,'r', linestyle='-', linewidth=plot_lw)
	# plot the best fit poly of velocity over time in blue
	plt.plot(list_x,poly_y, 'b', linestyle='-', linewidth=2)
	# setup the legend
	subplt.legend(['linear velocity error (m/s)', 'best fit polynomial'], fancybox=True, framealpha=0.5)
	subplt.axis([vtmin, vtmax, vmin, vmax])
	subplt.set_ylabel('linear velocity (m/s)')
	subplt.set_xlabel('time (s)')
	# name and save the plot
	fig.suptitle(name + "Velocity Error")
	fig.savefig(saver + " Velocity Error.png")
	plt.close()

def fix_data(bag_file):
	"""
	Repairs data in the turtlebot velocity topic by converting to a stamped message.
	Both topics (turtlebot velocity and vicon data) are written to a new file with -fixed appended to the name.

	Parameters:
		bag_file - file to be repaired
	Returns:
		repaired_file - name of the repaired bag file
	"""

	# setup repaired file name
	repaired_file = bag_file.replace(".bag", "_fixed.bag")  # Output bag path

	# the two topics to put in the new file
	topic1 = '/mobile_base/commands/velocity'  # Target topic you want to repair
	topic2 = '/vicon/turtlebot_traj_track/turtlebot_traj_track'
	
	# Open bags
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
	"""
	Main control loop for post processing the data. Finds bag files within the current directory to process.
	Note: current directory must be writable.

	Parameters:
		bag_file - file to be repaired
	Returns:
		repaired_file - name of the repaired bag file
	"""

	#  set the directory to current
	os.chdir(".")

	# setup CSV file for output data by clearing contents
	file = open("outdata.csv","w")
	file.write("Filename, v_error_rms, theta_error_rms, v_error_max, theta_error_max, v_error_avg, theta_error_avg,\n")
	file.close() 

	# get every *.bag in current directory
	for bag_file in glob.glob("*.bag"):

		# skip bags with "fixed" in name
		if "fixed" in bag_file:
			continue

		# print the bag file name currently being processed
		print(bag_file)
		# get data from the naming convention
		a = bag_file[:-4].split('_')
		# controller type in uppercase
		cntrllr = a[0].upper()
		# disturbance type
		disturbance = a[1].title()
		# trial number
		trial = a[2].title()

		# get the repaird file name
		repaired_file = fix_data(bag_file)

		# pass repaird file to processor and set x_data_range of 1.5
		pd = processor(repaired_file, x_data_range=1.5)
		# pass the pd to the plotter
		plotter(pd, cntrllr, disturbance, trial, bag_file[:-4])