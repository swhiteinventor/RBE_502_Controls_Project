# DFL_Controller
from math import cos, sin
def DFL_controller(controller, error_x, error_y, error_x_dot, error_y_dot, desired_x_dot_dot, desired_y_dot_dot, desired_v, current_theta):

	u1 = desired_x_dot_dot + controller.kp_1*error_x + controller.kd_1*error_x_dot
	u2 = desired_y_dot_dot + controller.kp_2*error_y + controller.kd_2*error_y_dot
	
	v = desired_v
	theta = (u2*cos(current_theta) - u1*sin(current_theta))/desired_v
	
	return -v, -theta
