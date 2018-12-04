from math import cos, sin, atan2

# PID_Controller
#def PID_controller(controller, error_v, error_v_dot, error_theta, error_theta_dot, desired_v):
def PID_controller(controller, error_x, error_y, error_x_dot, error_y_dot, time step):

	#v = desired_v + controller.kp_v*error_v + controller.ki_v*error_v + controller.kd_v*error_v
	#theta = desired_theta - controller.kp_theta*error_theta - controller.ki_theta*error_theta - controller.kd_theta*error_theta
	
	#v = controller.kp_v*error_v + controller.kd_v*error_v_dot + controller.ki_v*controller.v_area
	#omega = controller.kp_theta*error_theta + controller.kd_theta*error_theta_dot + controller.ki_theta*controller.theta_area

	x = self.kp_x*error_x + self.ki_x*self.area_x
	y = 
	theta = atan2(y/x)
	v = x*cos(theta) + y*sin(theta)
	omega = ()/time_step

	return v, omega
