# Launch Files

## [controls.launch](controls.launch)

Launch The Controller for use with Vicon or simulation
```
optional args:
	ctrl:=[ PID | DFL | NLF | MPC ]	: what controller to launch (default: PID)
	sim:=[ false | true ]		: use simulation (default: false)
	vicon:= [ false | true ] 	: launch vicon node (default: false)
	veloc:= [ 0.xxx ] 		: desired velocity to track in m/s (default: 0.325)
	theta:= [ xxx ] 		: desired theta to track in degrees (default: 0)

Use Cases:

	CTRL: select between the controllers
	veloc: desired velocity
	theta: desired angle (yaw)

	default | SIM | VICON | RESULT
	_________________________________________________________________________
	  Yes	|  F  |   F   | Must provide data or launch vicon externally
	  No	|  F  |   T   | Vicon node is launched too (slower to kill)
	  No	|  T  |   F   | Sub to data from Gazebo and Pub to Vicon topic
	  No	|  T  |   T   | Potential Sporadic Results: use at your own risk

Examples:

	roslaunch controls controls.launch ctrl:=DFL veloc:=0.25 theta:=30
		launches the DFL controller with desired velocity of 0.25m/s and theta of 30 degrees

	roslaunch controls controls.launch ctrl:=MPC
		launches the DFL controller with default velocity of 0.325m/s and default theta of 0 degrees
```

## [simulation.launch](simulation.launch)

Launch The Simulation Node
```
optional args:
	out:=[ false | true ]	: print output to screen (default: false)
```
The node will subscribe to /gazebo/model_states, format the data and publish it to the vicon topic

## [turtlebot.launch](turtlebot.launch)

Launch Turtlebot bringup or turtlebot in empty Gazebo world
```
optional args:
	sim:=[ false | true ]	: simulate the turtlebot (default: false)
```

## [vicon.launch](vicon.launch)

Duplicate of default vicon_bridge with optional IP argument
```
optional args:
	ip:=[ x.x.x.x ]	: Launch vicon with specified IP (default: 130.215.206.205)
```