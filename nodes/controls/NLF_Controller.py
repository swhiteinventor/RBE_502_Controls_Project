# NLF_Controller
from math import sin, cos, pi

def NLF_controller(controller, error_x, error_y, error_theta, desired_v, desired_omega):
	
	zeta = 2*controller.c1*((desired_omega)**2 + controller.c2*(desired_v)**2)**0.5
	v = zeta*error_x + desired_v*cos(error_theta)
	omega = desired_omega + zeta*desired_v*sin(error_theta)*error_y/error_theta + controller.c2*error_theta

	return v, omega
	