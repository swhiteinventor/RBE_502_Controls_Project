# DFL_Controller
import rospy

from math import cos, sin, pi
def DFL_controller(controller, error_x, error_y, error_x_dot, error_y_dot, desired_x_dot_dot, desired_y_dot_dot, desired_v, current_theta, error_v, error_theta):

	u1 = desired_x_dot_dot + controller.kp_1*error_x + controller.kd_1*error_x_dot
	u2 = desired_y_dot_dot + controller.kp_2*error_y + controller.kd_2*error_y_dot
	
	v = desired_v
	theta = (u2*cos(current_theta) - u1*sin(current_theta))/desired_v
	#print("u1=%.4f, u2=%.4f, cos()=%.4f, sin()=%.4f, theta = %.4f, e_v=%.4f, e_theta=%.4f" % (u1, u2, cos(current_theta), sin(current_theta), theta, error_v, error_theta*180/pi))

	return v, theta
