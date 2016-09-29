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

	@staticmethod
	def gaussian2(mu, sigma, x):
		""" calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		:param mu:    distance to the landmark
		:param sigma: standard deviation
		:param x:     distance to the landmark measured by the robot
		:return gaussian value
		"""

		# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


	def measurement_prob(self, measurement):
		""" Calculate the measurement probability: how likely a measurement should be
		:param measurement: current measurement
		:return probability
		"""

		prob = 1.0

		for i in range(len(landmarks)):
			dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
			prob *= self.gaussian2(dist, self.sense_noise, measurement[i])
		return prob



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

	def getParticle_likelihood(self):
		return self.particle_likelihood

	def setParticle_likelihood(self,value):
		if value >= 0.0:
			self.particle_likelihood = value
		else:
			print "Particle likelihoo out of bounds"

def visualization(robot, step, p, pr, weights):
    """ Visualization
    :param robot:   the current robot object
    :param step:    the current step
    :param p:       list with particles
    :param pr:      list of resampled particles
    :param weights: particle weights
    """

    plt.figure("Robot in the world", figsize=(15., 15.))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])

    # draw particles
    for ind in range(len(p)):

        # particle
        circle = plt.Circle((p[ind].x, p[ind].y), 1., facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(p[ind].x, p[ind].y, 2*cos(p[ind].orientation), 2*sin(p[ind].orientation), alpha=1., facecolor='#994c00', edgecolor='#994c00')
        plt.gca().add_patch(arrow)


    # draw resampled particles
    for ind in range(len(pr)):

        # particle
        circle = plt.Circle((pr[ind].x, pr[ind].y), 1., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(pr[ind].x, pr[ind].y, 2*cos(pr[ind].orientation), 2*sin(pr[ind].orientation), alpha=1., facecolor='#006600', edgecolor='#006600')
        plt.gca().add_patch(arrow)

    # fixed landmarks of known locations
    for lm in landmarks:
        circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
        plt.gca().add_patch(circle)

    # robot's location
    circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)

    # robot's orientation
    arrow = plt.Arrow(robot.x, robot.y, 2*cos(robot.orientation), 2*sin(robot.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)

    plt.savefig("output/figure_" + str(step) + ".png")
    plt.close()

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
	number_of_particles=1000 #particle at the beggining of the simulation
	list_of_particles=[]
	list_of_likelihoods=[]

	for i in range(number_of_particles):
		x = RobotClass()
		x.set_noise(0.005 ,0.0017, 0.1)
		x.set_sensor_vision_range(200.)
		#Since we do now know where the robot will start
		#lets put the particels uniformly distributed thought the map
		x.set_pose(random.uniform(0,wordSize),random.uniform(0,wordSize), random.uniform(0,2*pi))
		list_of_particles.append(x)

	number_of_steps=50

	for step in range(number_of_steps):

		# # First move the actual robot
		myrobot = myrobot.move(0.0,2.0)
		robotMeasurment = myrobot.sense_landmarks()
		plt.scatter(myrobot.x, myrobot.y, color='green')

		moved_particles=[]
		for particle_index in range(len(list_of_particles)):
			#move all the particles the same way as the actual robot
			moved_particles.append(list_of_particles[particle_index].move(0.0,2.0))
		list_of_particles = moved_particles

		#visualize the results
		if step%10 ==0:
			for particle_index in range(len(list_of_particles)):
				plt.scatter(list_of_particles[particle_index].x, list_of_particles[particle_index].y, color='blue')
			

		#Lets generate particle weights depending on the robot's measurments
		weights=[]

		for particle_index in range(len(list_of_particles)):
			weights.append(list_of_particles[particle_index].measurement_prob(robotMeasurment))
		print max(weights)

		#Resmpling with a sample probability proportional to the importance weight
		resampled_particles=[]
		index = int(random.random() * number_of_particles)
		beta = 0.0
		max_weight = max(weights)

		for i in range(len(list_of_particles)):
			beta += random.random() * 2.0 * max_weight

			while beta > weights[index]:
				beta -= weights[index]
				index = (index+1)%number_of_particles

			resampled_particles.append(list_of_particles[index])

		if max_weight != 0.0:
			list_of_particles = resampled_particles


		print len(list_of_particles), len(weights)
		plt.pause(0.01)

		


if __name__ == "__main__":
	main()
	plt.pause(10)







