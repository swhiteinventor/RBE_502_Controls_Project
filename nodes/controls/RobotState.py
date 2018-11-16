class RobotState():
	def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, t=0):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.t = t
	
	def set PositionAndRPY(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None):
		self.setPosition(x,y,z)
		self.setRPY(roll,pitch,yaw)

	def setPosition(self, x=None, y=None, z=None):
		if x!=None:
			self.x = x
		if y!=None:
			self.y = y
		if z!=None:
			self.z = z
	def setRPY(self, roll=None, pitch=None, yaw=None):
		if roll!=None:
			self.roll = roll
		if pitch!=None:
			self.pitch = pitch
		if yaw!=None:
			self.yaw = yaw