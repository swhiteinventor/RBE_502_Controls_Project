from math import cos, sin, atan2
from Robot_State import Robot_State, Data

# PID_Controller
#def PID_controller(controller, error_v, error_v_dot, error_theta, error_theta_dot, desired_v):
def PID_controller(controller, data):

	x = controller.kPID_x[0]*data.error_x + controller.kPID_x[1]*data.area_x + controller.kPID_x[2]*data.error_x_dot
	y = controller.kPID_y[0]*data.error_y + controller.kPID_y[1]*data.area_y + controller.kPID_y[2]*data.error_y_dot
	theta = atan2(y, x)
	v = x*cos(theta) + y*sin(theta)
	omega = (theta - data.current_theta)/data.time_step

	return v, omega
