class Robot_State():
	def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, t=0):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.t = t
	
	def set_position_rpy_t(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, t=None):
		self.set_position(x,y,z)
		self.set_rpy(roll,pitch,yaw)
		if t!=None:
			self.t = t

	def set_position(self, x=None, y=None, z=None):
		if x!=None:
			self.x = x
		if y!=None:
			self.y = y
		if z!=None:
			self.z = z

	def set_rpy(self, roll=None, pitch=None, yaw=None):
		if roll!=None:
			self.roll = roll
		if pitch!=None:
			self.pitch = pitch
		if yaw!=None:
			self.yaw = yaw
			
	def divide_by_time(self):
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