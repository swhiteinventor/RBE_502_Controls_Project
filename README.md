# Turtlebot Controllers (RBE 502)

## Tasks:
 - fix PID velocity
 - implement line tracking (2/3 done)
 - Tune all controller gains on real robot
 - Test implemention of data capture
 - Process one run (automated)
 - Test broad range of bumps, pick 2-4 to study
 - Test chosen bumps

## Repo Structure:
 - [launch/](launch/)
 	- ROS launch files for vicon, controllers, and simulation
 - [nodes/controls/](nodes/controls/)
 	- Controller base code
 	- Specific code for PID, DFL, and NLF controllers
 	- Additional code necessary for simulation
 	- Post processing code

## Setup / Prerequisites:
 - Data stream of turtlebot vicon data
 - Turtlebot2 repository setup
 - Gazebo for turtlebot2 setup
 - Controls (this repo) setup

## Running:

### Simulation:
To easily demonstrate the ability on simulation run the following commands in two seperate terminal windows:

```
	roslaunch controls turtlebot.launch sim:=true
	roslaunch controls controller.launch sim=:true
```

The 1st command will launch gazebo with the turtlbot in an empty world.

The second command will launch the controller while retrieving the fake vicon data from gazebo.

### Actual:
In three seperate terminals run the following commands, replacing <VICON IP> with your vicon IP address:

```
	roslaunch controls turtlebot.launch
	roslaunch controls vicon.launch ip:=<VICON IP>
	roslaunch controls controller.launch
```

### Testing other controllers:
[nodes/controls/controller.launch](controller.launch) supports multiple arguments so please see that file for more information. But to test other controllers pre-implemented add one the following to the launch command:
```
	ctrl:=DFL
	ctrl:=NLF
```
Note: the PID controller will run when none is specified.

## Additional Notes:
 - Simulation tested on 16.0x : Kinetic
 - Turtlebot tested on 14.0x : Indigo
 - Vicon data publishes to /vicon/turtle_traj_track/turtle_traj_track
 - vicon IP can be set in the vicon launch arguments