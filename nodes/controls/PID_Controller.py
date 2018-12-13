from math import cos, sin, atan2

# PID_Controller
def PID_controller(controller, data):

	x = controller.kPID_x[0]*data.error_x + controller.kPID_x[1]*data.area_x + controller.kPID_x[2]*data.error_x_dot
	y = controller.kPID_y[0]*data.error_y + controller.kPID_y[1]*data.area_y + controller.kPID_y[2]*data.error_y_dot
	theta = atan2(y, x)
	v = x*cos(theta) + y*sin(theta)*controller.PID_velocity_scale
	omega = (theta - data.current_theta)/data.time_step*controller.PID_omega_scale

	return v, omega
