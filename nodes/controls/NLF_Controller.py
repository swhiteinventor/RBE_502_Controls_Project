# NLF_Controller
from math import sin, cos, pi

def NLF_controller(controller, data):
	
	zeta = 2*controller.c[0]*((data.desired_omega)**2 + controller.c[1]*(data.desired_v)**2)**0.5
	v = zeta*data.error_x + data.desired_v*cos(data.error_theta)
	omega = data.desired_omega + zeta*data.desired_v*sin(data.error_theta)*data.error_y/data.error_theta + controller.c[1]*data.error_theta

	return v, omega
	