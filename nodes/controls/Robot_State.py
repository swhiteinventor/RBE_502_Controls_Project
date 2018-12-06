class Robot_State():
	def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, t=0):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.t = t
			
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

	def __str__(self):
		a = self

		return "Robot State:\n  x: %.4f y: %.4f z: %.4f\n  roll: %.4f pitch: %.4f yaw: %.4f\n  t: %.4f" % (a.x, a.y, a.z, a.roll, a.pitch, a.yaw, a.t)

class Data():
	def __init__(self):
		return None