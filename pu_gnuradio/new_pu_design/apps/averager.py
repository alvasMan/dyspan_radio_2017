class MovingAverageFilter:
	"""Simple moving average filter"""
 
	@property
	def avg(self):
		"""Returns current moving average value"""
		return self.__avg
 
	def __init__(self, n = 8, initial_value = 0):
		"""Inits filter with window size n and initial value"""
		self.__n = n
		self.__buffer = [initial_value/n]*n
		self.__avg = initial_value
		self.__p = 0
 
	def __call__(self, value):
		"""Consumes next input value"""
		self.__avg -= self.__buffer[self.__p]
		self.__buffer[self.__p] = value/self.__n
		self.__avg += self.__buffer[self.__p]
		self.__p = (self.__p  + 1) % self.__n
		return self.__avg
