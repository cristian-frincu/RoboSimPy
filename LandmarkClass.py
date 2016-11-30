class LandmarkClass:
	# A class that represents a landmark to a robot
	# This class is to be used by RobotClass to create a 
	# map using landmarks
	def __init__(self):
		#Absolute truth where the landmark is
		self.real_x=0.0
		self.real_y=0.0


		#Where the robot believes the landmark 
		#is located
		self.believed_x=0.0
		self.believed_y=0.0

		#A holder where the robot can use to keep
		#adding errorous values to, which will edventually
		#be averaged to get a better estimate of the location
		# of landmark
		self.measurment_sum_x=0.0
		self.measurment_sum_y=0.0
		#Since we are taking the distance measurments from the 
		# robot, and the robot is assumed to be at 0,0 at all 
		# times, we will get coordinates to the robot. Those
		# would be stored in here
		self.distance_to_robot_x=0.0
		self.distance_to_robot_y=0.0
