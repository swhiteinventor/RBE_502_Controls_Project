# Controller Files

## I/O

**Inputs:**
The current global position in x,y,z (meters)
The current global orientation in x,y,z,w (radians) (quaternian format)
The current timestamp of the data

**Outputs:**
V: forward velocity to command the turtlebot (m/s)
W: angular velocity to command the turtlebot (rad/s)

## Controller Types

### PID - Proportional Integral Derivative

### DFL - Dynamic Feedback Linearization

### NLF - Non-Linear Feedback

## Additional Files:

[controller.py](controller.py)
[Robot_State.py](Robot_State.py)
[post_processing.py](post_processing.py)
[Simulation.py](simulation.py)