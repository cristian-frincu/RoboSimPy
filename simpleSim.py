from RobotClass import RobotClass
import matplotlib.pyplot as plt
from math import *


# landmarks which can be sensed by the robot (in meters)
landmarks = [[20.0, 20.0]]

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

def visualize_robot_view(robot, step,landmarks,robot_name="0",path=[]):
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


    #robot's path
    if len(path)>0:
    	path.reverse()
        previous_pose = path[0]
        for pose in path:
            x_new = pose[0] - path[0][0]
            y_new = pose[1] - path[0][1]
            arrow = plt.Circle((x_new, y_new), 0.5, facecolor='#00FFFF', edgecolor='#000000')
            plt.gca().add_patch(arrow)
            previous_pose = pose


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

def map_error(trueLandmarks, beliefLandmarks):
	if len(trueLandmarks)!=len(beliefLandmarks):
		print "Number of landmarks differ, Error!"
		return 0
	error=0.0
	for i in range(len(trueLandmarks)):
		error+= sqrt((trueLandmarks[i][0]-beliefLandmarks[i][0])**2+(trueLandmarks[i][1]-beliefLandmarks[i][1])**2)

	return error

def main():
    myrobot= RobotClass()
    myrobot.set(50,50,0)

    FORWARD_SPEED=4
    TURN_ANGLE = 0.2

    #In this simulation, I only want to test the mapping
    #So we can assume we know with 100% certainty where our
    #robot is. distance readings should still have uncertainty
    myrobot.set_noise(0.00,0.0,4)
    robotTravel_x = 0
    robotTravel_y = 0
    z = myrobot.sense()

    landmark_belief=[]
    avg_x_landmark=0.0
    avg_y_landmark=0.0

    for step in range(20):
        myrobot.move(TURN_ANGLE,FORWARD_SPEED,return_new_state=False)
        z_angle =  myrobot.sense_angle(landmarks = landmarks,degrees=False)
        distance = myrobot.sense()
        landmark_description = zip(distance,z_angle)

        # Here we are getting the location of where the robot thinks the 
        # landmarks are.
        possible_landmark_loc=[]
        for mark in landmark_description:
            mark_x = mark[0]*cos(mark[1])
            mark_y = mark[0]*sin(mark[1])
            possible_landmark_loc.append([mark_x,mark_y])

        # We need to keep track of the distance traveled by the robot
        # in order to go back and be able to express the landmark position
        # in the same coordiante frame as the original landmarks
        robotTravel_x+= FORWARD_SPEED*cos(TURN_ANGLE)
        robotTravel_y+=FORWARD_SPEED*sin(TURN_ANGLE)

        believed_x_location = 50+possible_landmark_loc[0][0]+robotTravel_x
        believed_y_location = 50+possible_landmark_loc[0][1]+robotTravel_y


        # print "Believed Location:(x,y)",believed_x_location, believed_y_location
        # print "Measurment Error:",map_error(landmarks,possible_landmark_loc)

        #We are taking an average of the multiple measurments that way
        # the error from the measurment is cut down on
        avg_x_landmark+= believed_x_location
        avg_y_landmark+= believed_y_location

        visualize_robot_view(myrobot,step,possible_landmark_loc,path = myrobot.perceived_path)
        print "Step:",step,
    	print avg_x_landmark/(step+1), avg_y_landmark/(step+1)

if __name__ == "__main__":
    main()