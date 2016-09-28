from math import *
import random
import matplotlib.pyplot as plt


wordSize=100

landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
             [50.0, 20.0], [50.0, 80.0], [80.0, 80.0],
             [80.0, 20.0], [80.0, 50.0], [75.0, 55.0],
             [50.0, 25.0], [25.0, 25.0], [30.0, 40.0]]


class RobotClass():
	def __str__(self):
		return str(self.x)+", "+str(self.y)+", "+str(self.orientation)

	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.orientation = 0.0

		self.forward_noise = 0.0
		self.turn_noise = 0.0
		self.sense_noise = 0.0
		self.sensor_vision_range=0.0

	def set_pose(self,new_x,new_y,new_orientation):
		if new_x <= wordSize and new_y <= wordSize:
			self.x = new_x
			self.y = new_y
		else:
			print "New coordinated out of bounds"

		if new_orientation < 2*pi and new_orientation > 0:
			self.orientation = new_orientation
		else:
			print "New orientation out of bounds"


	def set_noise(self,new_forward_noise, new_turn_noise, new_sense_noise):
		self.forward_noise = new_forward_noise
		self.turn_noise = new_turn_noise
		self.sense_noise = new_sense_noise

	def set_sensor_vision_range(self,new_sensor_vision_range):
		if new_sensor_vision_range > 0.0:
			self.sensor_vision_range = new_sensor_vision_range
		else:
			print "New sensor vison range is negative"

	def sense_landmarks(self):
		# Calcualte the distance to the landmarks, and check if the 
		# installed sensor can possibly see them, if it can, then add 
		# the measurment to a list and return it
		# The sensor error is also added to the measurment
		z = []

		for i in range(len(landmarks)):
			dist = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
			dist += random.gauss(0.0, self.sense_noise)
			if dist <= self.sensor_vision_range:
				z.append(dist)

		return z

	def move(self,angle,distance):
		#Lets assume the robot can only move forward to cut down on the complexity

		if distance < 0.0:
			print "Robot can only move forward"

		new_orientation = self.orientation + float(angle) + random.gauss(0.0, self.turn_noise)
		new_orientation %= 2 * pi

		# Add some noise to the distance travelled
		new_distance= distance + random.gauss(0.0, self.forward_noise)
		new_x = self.x + (new_distance * cos(new_orientation))
		new_y = self.y + (new_distance * sin(new_orientation))

		#Check if the robot it getting out of bounds, if it does, make it
		#go thought the walls and come out on the other side

		new_x %= wordSize
		new_y %= wordSize

		#return a new robot at the desired posion, after all the probabilities have been
		#taken into account

		new_pose_robot = RobotClass()
		new_pose_robot.set_noise(self.forward_noise,self.turn_noise, self.sense_noise)
		new_pose_robot.set_sensor_vision_range(self.sensor_vision_range)
		new_pose_robot.set_pose(new_x, new_y, new_orientation)

		return new_pose_robot

	def gaussian(self,mu,sigma,x):
		exponentialTop = float((x-mu)**2)
		exponentialBottom = 2.0*sigma**2
		exponential = exp(-exponentialTop/exponentialBottom)
		bottom = sqrt(2.0*pi*sigma**2)
		return exponential/bottom

	def calculate_measurment_probability(self, measurments):
		#this might be a bit more complicated if the sensor vision does not 
		#see all the landmarks at all time
		probability = 1.0
		#At the mean, the normal distribution will give the max value of the function
		#I will use this as a normalizing factor, to get my likelyhood probabilities between
		#0 and 1
		normalizingDistributionValue = self.gaussian(0.0,self.sense_noise, 0.0) 
		for i in range(len(measurments)):
			actualDist = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
			# print measurments[i], actualDist, self.sense_noise, self.gaussian(actualDist,self.sense_noise, measurments[i])/normalizingDistributionValue
			probability *= self.gaussian(actualDist,self.sense_noise, measurments[i])/normalizingDistributionValue
		return probability

	def calculate_distance_between_robots(self,other_robot):
		distance = sqrt((self.x - other_robot.x)**2 + (self.y - other_robot.y)**2)
		return distance



def main():
	### introduction to the robot class
	plt.ion()

	#plot the landmarks
	for landmark in landmarks:
		plt.scatter(landmark[0],landmark[1],color='red')

	# create a robot
	myrobot = RobotClass()


	# set noise parameters
	#5mm error per move, .1 degree orientation per move, 10cm for measurment
	myrobot.set_noise(0.005 ,0.0017, 0.1)
	myrobot.set_sensor_vision_range(200.)
	myrobot.set_pose(50., 50., 1.25*pi)	



	#Start of particle filter implementation
	number_of_particles=250
	list_of_particles=[]

	for i in range(number_of_particles):
		x = RobotClass()
		x.set_noise(0.005 ,0.0017, 0.1)
		x.set_sensor_vision_range(200.)
		#Since we do now know where the robot will start
		#lets put the particels uniformly distributed thought the map
		x.set_pose(random.uniform(0,wordSize),random.uniform(0,wordSize), random.uniform(0,2*pi))
		list_of_particles.append(x)

	number_of_steps=1000

	for step in range(number_of_steps):
		# First move the actual robot
		myrobot = myrobot.move(0.0,2.0)
		

		high_likelihood_particles=[]
		particle_likelihood=[]

		# Then move all the particles in the same direction
		# and estimate the likelyhood of measurment
		for i in range(len(list_of_particles)):
			# plt.scatter(list_of_particles[i].x, list_of_particles[i].y, color='white')
			list_of_particles[i] = list_of_particles[i].move(0.0,2.0)
			plt.scatter(list_of_particles[i].x, list_of_particles[i].y, color='blue')
			

			#Add particles to the high likelyhood ones
			if random.uniform(0,1) > 0.5:
				high_likelihood_particles.append(list_of_particles[i])
				landmarkDistances = list_of_particles[i].sense_landmarks()
				measurment_likelihood = list_of_particles[i].calculate_measurment_probability(landmarkDistances)
				particle_likelihood.append(measurment_likelihood)
			# else:
			# 	plt.scatter(list_of_particles[i].x, list_of_particles[i].y, color='white')
			

		plt.scatter(myrobot.x, myrobot.y, color='green')


		MIN_PARTICLES=20
		particles_with_likelihood = [(x,(y/max(particle_likelihood))*MIN_PARTICLES) for (y,x) in zip(particle_likelihood,high_likelihood_particles)]
		
		for particle in particles_with_likelihood:
			if particle[1] > 0.5 :
				new_particle = particle[0]
				high_likelihood_particles.append(new_particle)
			# else:
				# plt.scatter(list_of_particles[i].x, list_of_particles[i].y, color='white')
			
		
		## Resampling based on the measurment likelihood
		# First start by matching the particle with its corresponding measurment likelihood
		if len(high_likelihood_particles)< MIN_PARTICLES:
			particles_with_likelihood = [(x,(y/max(particle_likelihood))*MIN_PARTICLES) for (y,x) in zip(particle_likelihood,high_likelihood_particles)]
			resampled_particles=[]
			for particle in particles_with_likelihood:
				for i in range(int(particle[1])):
					new_particle =particle[0]
					# print particle[0].orientation
					new_x = (particle[0].x + random.uniform(0,50))%100
					new_y = (particle[0].y + random.uniform(0,50))%100
					new_orientation = (particle[0].orientation + random.uniform(0,pi*2))%pi*2
					new_particle.set_pose(new_x, new_y, new_orientation)
					resampled_particles.append(new_particle)
		else:
			resampled_particles= high_likelihood_particles
			# print number_of_particles_to_resampled



		list_of_particles = resampled_particles
		print "Particles in the system:",len(list_of_particles)


		#the measurments taken with the sensors on the robot
		z = myrobot.sense_landmarks() 


		#Measurment of how close the particles are to the actual robot
		distances=[]
		for i in list_of_particles:
			distances.append(myrobot.calculate_distance_between_robots(i))

		print "Closest Particle at:",min(distances)



		#simulate the same measurment for all the particles that you have
		plt.pause(0.001)


if __name__ == "__main__":
	main()
	plt.pause(1000)







