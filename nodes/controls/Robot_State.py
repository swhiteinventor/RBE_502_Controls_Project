class Robot_State():
	"""
	Class to contain state information of the Robot such as positional and rotational xyz along with the time stamp for the data.
	"""

	def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, t=0):
		"""
		Initiallizes Robot_State object
		Unset variables are default to 0

		Parameters:
			x - positional x data
			y - positional y data
			z - positional z data
			roll - rotational x data
			pitch - rotational y data
			yaw - rotational z data
			t - time data
		Returns:
			Robot_State - with filled variables
		"""
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.t = t
			
	def divide_by_time(self):
		"""
		Divides each variable of Robot_State by the Robot_State time field
		The time field is returned without division

		Parameters:
			None
		Returns:
			Robot_State - variables will be divided by time
		"""
		a = self
		r = Robot_State()
			
		r.x = a.x/a.t
		r.y = a.y/a.t
		r.z = a.z/a.t
		r.roll = a.roll/a.t
		r.pitch = a.pitch/a.t
		r.yaw = a.yaw/a.t
		r.t = a.t

		return r

	def __sub__(self, b):
		"""
		Overrides the subtract method for Robot_State
		Robot_State = a - b

		Parameters:
			Robot_State - (autohandled) b portion of (a-b)
		Returns: 
			Robot_State - after subtraction
		"""
		a = self
		r = Robot_State()

		r.x = a.x - b.x
		r.y = a.y - b.y
		r.z = a.z - b.z
		r.roll = a.roll - b.roll
		r.pitch = a.pitch - b.pitch
		r.yaw = a.yaw - b.yaw
		r.t = a.t - b.t

		return r

	def __str__(self):
		"""
		Overrides the as string method for Robot_State

		Parameters:
			None
		Returns:
			string - Contains each variable of Robot_State to 4 decimal places
		"""
		a = self

		return "Robot State:\n  x: %.4f y: %.4f z: %.4f\n  roll: %.4f pitch: %.4f yaw: %.4f\n  t: %.4f" % (a.x, a.y, a.z, a.roll, a.pitch, a.yaw, a.t)

class Data():
	"""
	Data wrapper for passing data to controller implementations
	"""

	def __init__(self):
		"""
		Initiallize the Data wrapper with no values

		Parameters:
			None
		Returns:
			None
		"""
		return None