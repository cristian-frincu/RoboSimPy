#
# This module contains some basics about the Particle filter (based on the Udacity class by Sebastian Thrun)
#
# The current example uses the class 'robot', this robot lives in the 2D world
# with size of 100 m x 100 m and can see four landmarks with given coordinates
#


from math import *
import random
import matplotlib.pyplot as plt
import numpy as np

from RobotClass import RobotClass



# landmarks which can be sensed by the robot (in meters)
landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
             [50.0, 20.0], [50.0, 80.0], [80.0, 80.0],
             [80.0, 20.0], [80.0, 50.0], [85.0, 20.0],
             [20.0, 15.0], [45.0, 45.0]]

# size of one dimension (in meters)
world_size = 100.0



def evaluation(r, p):
    """ Calculate the mean error of the system
    :param r: current robot object
    :param p: particle set
    :return mean error of the system
    """

    sum = 0.0

    for i in range(len(p)):

        # the second part is because of world's cyclicity
        dx = (p[i].x - r.x + (world_size/2.0)) % \
             world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % \
             world_size - (world_size/2.0)
        err = sqrt(dx**2 + dy**2)
        sum += err

    return sum / float(len(p))



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
    # create a robot for the particle filter demo
    myrobot = RobotClass()
    myrobot = myrobot.move(0.1, 5.0)
    z = myrobot.sense()

    RESAMPLE=True
    number_of_particles = 500
    list_of_particles = []    


    steps = 50  # particle filter steps
    previous_step_ratio=0
    for step in range(steps):

        # create a set of particles
        for i in range(number_of_particles):
            x = RobotClass()
            x.set_noise(0.05, 0.05, 5.0)
            # x.set_noise(0.5, 0.25, 5.0)
            list_of_particles.append(x)


        # move the robot and sense the environment after that
        myrobot = myrobot.move(0.1, 5.)
        measurment = myrobot.sense()

        # now we simulate a robot motion for each of these particles
        moved_particles = []

        for i in range(number_of_particles):
            moved_particles.append( list_of_particles[i].move(0.1, 5.) )

        list_of_particles = moved_particles

        # generate particle weights depending on robot's measurement
        weights = []

        for i in range(number_of_particles):
            weights.append(list_of_particles[i].measurement_prob(measurment))

        normalized_weights=[]
        max_weight = max(weights)
        for weight in weights:

            normalized_weights.append(weight/max_weight)

        # print normalized_weights

        # plt.ion()
      #  vfig = plt.figure()
      #  ax = fig.add_subplot(111)
      #  n, bins, rectangles = ax.hist(normalized_weights, 50, normed=True)
      #  fig.canvas.draw()
        # plt.pause(0.05)

        measurement_ratio = min(weights)/max(weights)
        if measurement_ratio < 0.01:
            RESAMPLE = True
        else:
            RESAMPLE = False

        #plt.title('Particle Weight Histogram, step:' + str(step) +" Resample:" + str(RESAMPLE))
        #plt.savefig("output/wtHist_" + str(step) + ".png")
        #plt.close()

        # resampling with a sample probability proportional to the importance weight
        #have the ability to resample, or just ignore the resampling and keep
        #the same particles
        resampled_particles = []
        if RESAMPLE:
            index = int(random.random() * number_of_particles)
            beta = 0.0
            mw = max(weights)

            for i in range(number_of_particles):
                beta += random.random() * 2.0 * mw

                while beta > weights[index]:
                    beta -= weights[index]
                    index = (index + 1) % number_of_particles
                resampled_particles.append(list_of_particles[index])
            list_of_particles = resampled_particles
        else:
            #When we ignore the resampling step, we can just pass the same
            #particles forward to the next step
            resampled_particles = list_of_particles

        number_of_particles = len(list_of_particles)

        # visualize the current step
        visualization(myrobot, step, moved_particles, resampled_particles, weights)


        print 'Step = ', step, ', Ratio = ', measurement_ratio,", Length = ", len(list_of_particles), "Evaluation:", evaluation(myrobot,resampled_particles), "Resample:",RESAMPLE

if __name__ == "__main__":
    main()
