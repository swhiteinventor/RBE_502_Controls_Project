# NLF_Controller
from math import sin, cos, pi

def NLF_controller(controller, error_x, error_y, error_theta, desired_v, desired_theta):
	#desired_theta = desired_theta + pi
	
	#zeta = 2*controller.c1*((desired_v)**2 + (desired_theta)**2)**0.5
	zeta = 2*controller.c1*((desired_v)**2 + (desired_theta)**2)**0.5
	v = zeta*error_x + desired_v*cos(error_theta)
	theta = desired_theta + zeta*desired_v*sin(error_theta)*error_y/error_theta + controller.c2*error_theta

	return v, theta

	#STATUS:
	#it tracks the trajectory facing the wrong direction
	#tracks velocity changes correctly
	