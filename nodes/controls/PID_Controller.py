# PID_Controller
def PID_controller(controller, error_v, error_theta, desired_v):

	#v = desired_v + controller.kp_v*error_v + controller.ki_v*error_v + controller.kd_v*error_v
	#theta = desired_theta - controller.kp_theta*error_theta - controller.ki_theta*error_theta - controller.kd_theta*error_theta

	v =  desired_v + controller.kp_v*error_v
	theta = controller.kp_theta*error_theta


	return v, theta

	#STATUS:
	#tracks velocity changes correctly