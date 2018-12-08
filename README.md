# Turtlebot Controllers (RBE 502)

## Tasks:
 - Tune MPC controller/update code
 - Test implemention of data capture
 - Get all data for one controller
 - Revise plots and data collection as needed
 - Get data for all controllers
 - Create Presentation
 - Write report

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
roslaunch controls controls.launch sim=:true
```

The 1st command will launch gazebo with the turtlbot in an empty world.

The second command will launch the controller while retrieving the fake vicon data from gazebo.

Note: the PID controller will run when no controller is specified. See below for how to specify the controller.


### Actual:
In three seperate terminals run the following commands, replacing <VICON IP> with your vicon IP address:

```
roslaunch controls turtlebot.launch
roslaunch controls vicon.launch ip:=<VICON IP>
roslaunch controls controls.launch
```

### Testing other controllers (in simulation or or actual):
[controller.launch](nodes/controls/controls.launch) supports multiple arguments so please see that file for more information. But to test other controllers pre-implemented add one the following to the launch command:
```
ctrl:=PID
ctrl:=DFL
ctrl:=NLF
ctrl:=MPC
```
Note: the PID controller will run when no controller is specified.

Descriptions:
- PID is a Proportional Integral Derivative Controller
- DFL is a Dynamic Feedback Linearization Controller
- NLF is a Nonlinear Feedback Controller
- MPC is a Model Predictive Controller

## Tuning Controllers:
All controller gains were obtained by tuning the controller prior to testing with terrain disturbances. The tuining procedure consisted of requesting a 0.325 m/s velocity at a 0 degree heading. The robot would start initially at a 45 degree heading or a -45 degree heading. The controller gains were changed until critical dampening of the trajectory tracking was achieved with a settling time of 2 s, which is a distance of 0.65 m from the initial position. This procedure was designed so that there was a standard metric to which all controllers were tuned to allow us to compare the transient performance of each controller with unexpected terrain disturbances.

## Results:
To be added.

## Additional Notes:
 - Simulation tested on 16.0x : Kinetic
 - Turtlebot tested on 14.0x : Indigo
 - Vicon tested on 3.4.0 with 8 ceiling mounted cameras and 2 tripod mounted cameras
 - Vicon data publishes to /vicon/turtlebot_traj_track/turtlebot_traj_track
 - vicon IP can be set in the vicon launch arguments