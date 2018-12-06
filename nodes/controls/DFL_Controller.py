# DFL_Controller
from math import cos, sin, pi
from Robot_State import Robot_State, Data

def DFL_controller(controller, data):

	u1 = data.desired_x_dot_dot + controller.kPD_1[0]*data.error_x + controller.kPD_1[1]*data.error_x_dot
	u2 = data.desired_y_dot_dot + controller.kPD_2[0]*data.error_y + controller.kPD_2[1]*data.error_y_dot
	
	v = data.desired_v
	omega = (u2*cos(data.current_theta) - u1*sin(data.current_theta))/data.desired_v

	return v, omega
