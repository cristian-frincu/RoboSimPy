import random
import numpy as np
import matplotlib.pyplot as plt
from RobotClass import RobotClass
from math import *

def visualization(robots, step):
    """ Visualization
    :param robot:   the current robot object
    :param step:    the current step
    """

    world_size = 100
    plt.figure("Robot in the world", figsize=(15., 15.))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])


    # robot's location
    for robot in robots:
	    circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
	    plt.gca().add_patch(circle)

    # robot's orientation
    for robot in robots:
	    arrow = plt.Arrow(robot.x, robot.y, 2*cos(robot.orientation), 2*sin(robot.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
	    plt.gca().add_patch(arrow)

    plt.savefig("output/figure_" + str(step) + ".png")
    plt.close()



def main():
	robots=[]
	numOfRobots=3
#initiallize the location of the robots in a systematic way
# make it so the robots starts in a line, all close to eachother
	for i in range(numOfRobots):
		robot = RobotClass()
		robot.x = i * 15 + 30
		robot.y = 2.5 * i + 5
		robot.orientation = 3.14/2
		robot.forward_noise = 0.5
		robot.turn_noise = 0.3 
		robots.append(robot)
		
#The basic algorithm is that it will move the robot which is behind
#the other ones in line

	numOfSteps=15
	robotToMove=3 # which robot to move first
	for step in range(numOfSteps*numOfRobots):
		visualization(robots,step)
		toMove = min(robots, key=lambda robot: robot.y)
		for i in range(numOfRobots):
			if robots[i]== toMove:
#An interesting limitation of python, is that you cannot run range(n).remove(m) in one line
#since that would oddly return None. When done in two lines, it works as expected
				robotsInFront = range(numOfRobots)
				robotsInFront.remove(i)
#We are getting a list of all the possible robots. Now we remove the robot
#that is at the back
				print "Move:",i,"Stay:",robotsInFront
				
				spaceBetweenFrontRobots = robots[robotsInFront[0]].x - robots[robotsInFront[1]].x
				xLocationWhereToTravel = robots[robotsInFront[0]].x  + spaceBetweenFrontRobots/2.0
				robots[i] = toMove.move(0 , 5.0)


if __name__=="__main__":
	main()
