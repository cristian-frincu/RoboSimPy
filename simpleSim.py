from RobotClass import RobotClass
import matplotlib.pyplot as plt
from math import *


# landmarks which can be sensed by the robot (in meters)
landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
             [50.0, 20.0]]

# size of one dimension (in metelandmarks[i][0]rs)
world_size = 100.0




def visualization(robot, step, p, pr,landmarks):
    """ Visualization
    :param robot:   the current robot object
    :param step:    the current step
    :param p:       list with particles
    :param pr:      list of resampled particles
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

def visualize_robot_view(robot, step,landmarks,robot_name="0"):
    """ Visualization
    :param robot:   the current robot object
    :param step:    the current step
    :param landmarks:    list of landmarks
    :param robot_name:   used to identify the different robots
    """

    plt.figure("Robot in the world", figsize=(15., 15.))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [-world_size, world_size, -world_size, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(-int(world_size), int(world_size), 5)])
    plt.yticks([i for i in range(-int(world_size), int(world_size), 5)])

    # fixed landmarks of known locations
    for lm in landmarks:
        circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
        plt.gca().add_patch(circle)

    # robot's location
    # circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    circle = plt.Circle((0, 0), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)

    # robot's orientation
    # arrow = plt.Arrow(robot.x, robot.y, 2*cos(robot.orientation), 2*sin(robot.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    arrow = plt.Arrow(0, 0, 2*cos(robot.orientation), 2*sin(robot.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)


    plt.savefig("output/view_"+str(robot_name)+"_step_" + str(step) + ".png")
    plt.close()

def main():
    myrobot= RobotClass()
    myrobot.set(50,50,0)

    #In this simulation, I only want to test the mapping
    #So we can assume we know with 100% certainty where our
    #robot is. distance readings should still have uncertainty
    myrobot.set_noise(0.05,0.5,0.5)
    z = myrobot.sense()
    for step in range(5):
        myrobot.move(0.1,2,return_new_state=False)
        z_angle =  myrobot.sense_angle(landmarks = landmarks,degrees=False)
        distance = myrobot.sense()
        landmark_description = zip(distance,z_angle)

        possible_landmark_loc=[]
        for mark in landmark_description:
            mark_x = mark[0]*cos(mark[1])
            mark_y = mark[0]*sin(mark[1])
            possible_landmark_loc.append([mark_x,mark_y])

        print myrobot.perceived_path
        visualization(myrobot,step,[myrobot],[myrobot],landmarks)
        visualize_robot_view(myrobot,step,possible_landmark_loc)




if __name__ == "__main__":
    main()