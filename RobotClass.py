from math import *
import random


# landmarks which can be sensed by the robot (in meters)
# [x,y]
landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
             [50.0, 20.0], [50.0, 80.0], [80.0, 80.0],
             [80.0, 20.0], [80.0, 50.0], [85.0, 20.0],
             [20.0, 15.0]]

# size of one dimension (in meters)
world_size = 100.0


class RobotClass:
    """ Class for the robot model used in this demo """
    _init_already = False

    def __init__(self):
        self.x = random.random() * world_size           # robot's x coordinate
        self.y = random.random() * world_size           # robot's y coordinate
        self.orientation = random.random() * 2.0 * pi   # robot's orientation
        self.path = [] # the path the robot has actually traveled


        self.perceived_x = 0.0 # robots x coordinate based on odometry
        self.perceived_y = 0.0 # robots y coordinate based on odometry
        self.perceived_path = [] # the path the robot thinks has traveled

        self.forward_noise = 0.0   # noise of the forward movement
        self.turn_noise = 0.0      # noise of the turn
        self.sense_noise = 0.0     # noise of the sensing


    def set(self, new_x, new_y, new_orientation):
        """ Set robot's initial position and orientation
        :param new_x: new x coordinate
        :param new_y: new y coordinate
        :param new_orientation: new orientation
        """

        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)


    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        """ Set the noise parameters, changing them is often useful in particle filters
        :param new_forward_noise: new noise value for the forward movement
        :param new_turn_noise:    new noise value for the turn
        :param new_sense_noise:  new noise value for the sensing
        """

        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)


    def sense(self):
        """ Sense the environment: calculate distances to landmarks
        :return measured distances to the known landmarks
        """

        z = []

        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            z.append(dist)

        return z

    def sense_angle(self,degrees=True, landmarks= landmarks):
        # Sense the angle from the robot to each of the landmarks
        # The angle = tan-1(m) where m is the slope of the line between
        # the two points
        # m = (y2-y1)/(x2-x1) where 1 is the landmark and 2 is the robot
        z_angle = []

        for i in range(len(landmarks)):
            #If the div by zero does happen, it means we are standing on the 
            #landmark, in which case there is no angle to the landmark
            if (self.x-landmarks[i][0]) != 0:
                slope = abs((self.y-landmarks[i][1])/(self.x-landmarks[i][0]))
            else:
                slope = abs((self.y-landmarks[i][1])/(0.00000001)) # to avoid div by zero
            
            angle = atan(slope) #atan returns in radians

            delta_y = (self.y-landmarks[i][1])
            delta_x = (self.x-landmarks[i][0])
            
            #This is done in order to get all the range readings
            #In the regular coordinate system, where zero points
            #to the right in a graph, and all measurments are
            #angles from origin 
            if(delta_y< 0 and delta_x <0):
                angle = angle
            elif(delta_y<0 and delta_x > 0):
                angle = pi-angle
            elif(delta_y > 0 and delta_x > 0):
                angle = pi+angle
            elif(delta_y > 0 and delta_x < 0):
                angle = 2*pi - angle

            if degrees == True:
                #Append the angle in degrees
                z_angle.append(angle*57.295779513)
            else:
                #Append the angle in radians
                z_angle.append(angle)


        return z_angle 


    def move(self, turn, forward, return_new_state=True):
        """ Perform robot's turn and move
        :param turn:    turn command
        :param forward: forward command
        :return robot's state after the move (optional)
        If return_new_state is set to False, it will move the current
        particle and will not create a new one
        """

        if forward < 0:
            raise ValueError('Robot cannot move backwards')

        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)

        #The path the robot has traveled in reallity
        self.path.append([x,y])


        #For the perceived location, I will not add any noise
        #this is the value you would tell the robot to travel,
        #We cannot measure the noise
        dist = float(forward)
        perceived_x = self.perceived_x + (cos(orientation) * dist)
        perceived_y = self.perceived_y + (sin(orientation) * dist)

        #Also append the new point to the path
        self.perceived_path.append([perceived_x,perceived_y])


        # cyclic truncate
        x %= world_size
        y %= world_size

        if return_new_state:
            # set particle
            res = RobotClass()
            res.set(x, y, orientation)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

            return res
        else:
            self.x = x
            self.y = y
            self.orientation = orientation



    @staticmethod
    def gaussian(mu, sigma, x):
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
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        return prob


    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
