# Launch Files

## [controls.launch](controls.launch)

Launches The Controller for use with Vicon or simulation
```
**optional args:**
	ctrl:=[ PID | DFL | NLF ]	: what controller to launch (default: PID)
	sim:=[ false | true ]		: use simulation (default: false)
	vicon:= [ false | true ] 	: launch vicon node (default: false)

**Use Cases:**
	CTRL: select between the controllers

	**default | SIM | VICON | RESULT**
	_________________________________________________________________________
**	  Yes	|  F  |   F   | Must provide data or launch vicon externally**
	  No	|  F  |   T   | Vicon node is launched too (slower to kill)
	  No	|  T  |   F   | Sub to data from Gazebo and Pub to Vicon topic
	  No	|  T  |   T   | Potential Sporadic Results: use at your own risk
```

## [simulation.launch](simulation.launch)

Launch The Simulation Node
```
The node will subscribe to /gazebo/model_states, format the data and publish it to the vicon topic
**optional args:**
	out:=[ false | true ]	: print output to screen (default: false)
```

## [turtlebot.launch](turtlebot.launch)

Launch Turtlebot bringup or turtlebot in empty Gazebo world
```
**optional args:**
	sim:=[ false | true ]	: simulate the turtlebot (default: false)
```

## [vicon.launch](vicon.launch)

Duplicon of default vicon_bridge with optional IP argument
```
**optional args:**
	ip:=[ x.x.x.x ]	: Launch vicon with specified IP (default: 130.215.206.205)
```