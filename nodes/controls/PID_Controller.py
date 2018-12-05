from math import cos, sin, atan2

# PID_Controller
#def PID_controller(controller, error_v, error_v_dot, error_theta, error_theta_dot, desired_v):
def PID_controller(controller, error_x, error_y, error_x_dot, error_y_dot, time_step, current_theta):

	#v = desired_v + controller.kp_v*error_v + controller.ki_v*error_v + controller.kd_v*error_v
	#theta = desired_theta - controller.kp_theta*error_theta - controller.ki_theta*error_theta - controller.kd_theta*error_theta
	
	#v = controller.kp_v*error_v + controller.kd_v*error_v_dot + controller.ki_v*controller.v_area
	#omega = controller.kp_theta*error_theta + controller.kd_theta*error_theta_dot + controller.ki_theta*controller.theta_area

	x = controller.kp_x*error_x + controller.ki_x*controller.area_x + controller.kd_x*error_x_dot
	y = controller.kp_y*error_y + controller.ki_y*controller.area_y + controller.kd_y*error_y_dot
	theta = atan2(y, x)
	v = x*cos(theta) + y*sin(theta)
	omega = (theta - current_theta)/time_step

	return v, omega
