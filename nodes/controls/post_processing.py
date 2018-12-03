#!/usr/bin/python
import yaml
import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

if (len(sys.argv) == 2) and (sys.argv[1] == '--help'):
   print "Usage: plot_bag file_name <\"use_ground_truth\"> <robot namespace> <start_time> <end_time> "
   print "  plot_bag 2015-01-15-10-13-41.bag  0 10.4 20.5"
   print "  optional arguments must be in order "
   print "    set start time to starting time in seconds (default=0)"
   print "    set end time in seconds (default=final time in bag)"


file_name = 'log.bag'
if (len(sys.argv) > 1):
	file_name = sys.argv[1]

start_time = 0
end_time   = 9E9

if (len(sys.argv) > 2):
	start_time = float(start_time)

if (len(sys.argv) > 3):
	end_time = float(end_time)

print "Read bag file ..."
bag = rosbag.Bag(file_name, 'r')

print "Get topic info..."
info_dict = yaml.load(bag._get_yaml_info())
print(info_dict['topics'])

print "Get list ..."
topic_list = []
cmd_msgs = 0
state_msgs = 0

# ROS Topics
vicon_msgs		= []
cmd_vel_msgs	= []
clock_msgs		= []

# Time Data
clock_topic = '/clock'
time 		= []

# Vicon Data
vicon_topic = '/vicon/turtlebot_traj_track/turtlebot_traj_track'
vicon_time	= []

vicon_px	= []
vicon_py	= []
vicon_pz	= []

vicon_ox	= []
vicon_oy	= []
vicon_oz	= []
vicon_ow	= []

# CMD Data
cmd_vel_topic = '/mobile_base/commands/velocity'
cmd_time	= []

cmd_vx 		= []
cmd_wz		= []

for topic_info in info_dict['topics']:
	topic = topic_info['topic']
	topic_list.append(topic)

	if topic == clock_topic:
		clock_msgs = topic_info['messages']
	if topic == vicon_topic:
		vicon_msgs = topic_info['messages']
	if topic == cmd_vel_topic:
		cmd_vel_msgs = topic_info['messages']

print topic_list

print "Message counts:"
print "  clock msgs="+str(  clock_msgs	)
print "  vicon msgs="+str(  vicon_msgs	)
print "  cmd_vel msgs="+str(  cmd_vel_msgs	)

print "Process messages ..."
time_base = -1;
jnt = 16

print "	Process clock data ..."
if clock_msgs:
	time = [0 for x in xrange(clock_msgs)]

	pt = 0

	for topic, msg, t0 in bag.read_messages(topics=clock_topic):
			if (time_base < 0):
				time_base = msg.clock.secs + msg.clock.nsecs/1.0E9
				print "time base =",time_base

			
			time[pt] = (msg.clock.secs + msg.clock.nsecs/1.0E9 - time_base)

			pt = pt + 1

	start_time = min(time)
	end_time = min(end_time, max(time))

	print "Clock start time ="+str(min(time))
	print "Set end time ="+str(end_time)

if (cmd_vel_msgs):
	cmd_vx = [0 for x in xrange(cmd_vel_msgs)]
	cmd_wz = cmd_vx
	# cmd vel

	pt = 0

	for topic, msg, t0 in bag.read_messages(topics=cmd_vel_topic):
		cmd_vx[pt]   = msg.linear.x
		cmd_wz[pt]   = msg.angular.z

		pt = pt + 1

	#print "Cmd start time ="+str(min(time_cmd))
	#print "Cmd end time ="+str(max(time_cmd))

if (vicon_msgs):
	vicon_time = [0 for x in xrange(vicon_msgs)]
	vicon_px    = vicon_time
	vicon_py    = vicon_time
	vicon_pz    = vicon_time
	vicon_ox    = vicon_time
	vicon_oy    = vicon_time
	vicon_oz    = vicon_time
	vicon_ow    = vicon_time

	pt = 0

	for topic, msg, t0 in bag.read_messages(topics=vicon_topic):
			vicon_time[pt] = (msg.header.stamp.to_sec() - time_base)
			vicon_px[pt]    = msg.transform.translation.x
			vicon_py[pt]    = msg.transform.translation.y
			vicon_pz[pt]    = msg.transform.translation.z
			vicon_ox[pt]    = msg.transform.rotation.x
			vicon_oy[pt]    = msg.transform.rotation.y
			vicon_oz[pt]    = msg.transform.rotation.z
			vicon_ow[pt]    = msg.transform.rotation.w

			pt = pt + 1

	print "vicon start time ="+str(min(vicon_time))
	print "vicon end time ="+str(max(vicon_time))

print "Close bag!"
bag.close()


cmd_vx = cmd_vx[:len(time)]
cmd_wz = cmd_wz[:len(time)]

time = time[:len(cmd_vx)]
time = time[:len(cmd_wz)]
print len(time)
print len(cmd_wz)

if (cmd_vel_msgs):
	print "  Plot commands ..."
	tmin0 = min(time)-0.2
	tmax0 = max(time)+0.2
	vxmin0= min(cmd_vx)-0.2
	vxmax0= max(cmd_vx)+0.2
	wzmin0= min(cmd_wz)-0.2
	wzmax0= max(cmd_wz)+0.2

	print "  tmin=",str(tmin0),"  tmax=",str(tmax0)

	fig_cmd = plt.figure()
	ax_cmd = fig_cmd.add_subplot(211)
	ax_cmd.hold(True)
	ax_cmd.plot(time,cmd_vx,'r')
	ax_cmd.plot(time,cmd_wz,'g*')
	ax_cmd.axis([tmin0, tmax0, vxmin0, vxmax0])

	ax_cmd.set_ylabel('m/s rad/s')
	ax_cmd.set_xlabel('time')
	ax_cmd.legend(['vx_cmd (m/s)', 'wz_cmd (rad/s)'])
	fig_cmd.suptitle("Commands")


if (vicon_msgs):
	print "  Plot vicon ..."
	tmin0 = min([min(vicon_time),min(time)])-0.2
	tmax0 = max([max(vicon_time),max(time)])+0.2
	pmin0= min([min(vicon_px), min(vicon_py), min(vicon_pz)])-0.2
	pmax0= max([max(vicon_px), max(vicon_py), max(vicon_pz)])+0.2

	print "  tmin=",str(tmin0),"  tmax=",str(tmax0)

	fig_cmd = plt.figure()
	ax_cmd = fig_cmd.add_subplot(211)
	ax_cmd.hold(True)
	ax_cmd.plot(vicon_time,vicon_px,'r')
	ax_cmd.plot(vicon_time,vicon_py,'g*')
	ax_cmd.plot(vicon_time,vicon_pz,'b')
	ax_cmd.axis([tmin0, tmax0, pmin0, pmax0])

	ax_cmd.set_ylabel('meters')
	ax_cmd.set_xlabel('time')
	ax_cmd.legend(['px_actual (m)', 'py_actual (m)', 'pz_actual (m)'])
	fig_cmd.suptitle("Vicon Data")


print "Show plot..."
plt.show()