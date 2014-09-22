import serial

class Claw:
	def __init__(self):
		pass
	
	def startup(self, port, baud=9600):
		""" initiate connection to the microcontroller """
		self.s = serial.Serial(port, baud)
		self.s.readline()			# block until ready
		
	def shutdown(self):
		self.s.close()
		
	def grip(self, angle):
		self.s.write('c' + str(angle) + '\n')
		self.s.readline()
		
	def tilt(self, angle):
		self.s.write('t' + str(angle) + '\n')
		self.s.readline()
