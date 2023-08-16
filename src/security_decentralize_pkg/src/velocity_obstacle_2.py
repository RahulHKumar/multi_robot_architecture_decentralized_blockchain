#!/usr/bin/env python

import rospy
from security_decentralize_pkg.msg import pp1
# from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray

from multi_robot_plot import plot_robot_and_obstacles
from create_obstacles import create_obstacles
from control import compute_desired_velocity
import numpy as np


SIM_TIME = 5.
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
#5/0.1=50 time samples where the velocities are being calculated
ROBOT_RADIUS = 0.5
VMAX = 2
VMIN = 0.2

start = np.array([2, 5, 0, 0])
goal = np.array([9, 2, 0, 0])
#the above arrays are (x,y,_,_)

robot_state = start
robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS)) #4 X NUMBER_OF_TIMESTEPS uninitialized matrix
i=0
c=0
obstacle_to_be_passed = np.zeros((4,NUMBER_OF_TIMESTEPS,2))

def simulate():
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

    start = np.array([5, 0, 0, 0])
    goal = np.array([5, 10, 0, 0])
    #the above arrays are (x,y,_,_)

    robot_state = start
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS)) #4 X NUMBER_OF_TIMESTEPS uninitialized matrix
    print(type(obstacles[:, 0, :]))
    for i in range(NUMBER_OF_TIMESTEPS):
        v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX) #returns normalized velocity vector
        control_vel = compute_velocity(
            robot_state, obstacles[:, i, :], v_desired)
        robot_state = update_state(robot_state, control_vel)
        robot_state_history[:4, i] = robot_state

    plot_robot_and_obstacles(
        robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)


def compute_velocity(robot, obstacles, v_desired):
    pA = robot[:2]
    vA = robot[-2:]
    # Compute the constraints
    # for each velocity obstacles
    number_of_obstacles = np.shape(obstacles)[1]
    Amat = np.empty((number_of_obstacles * 2, 2))
    bvec = np.empty((number_of_obstacles * 2))
    for i in range(number_of_obstacles):
        obstacle = obstacles[:, i]
        pB = obstacle[:2]
        vB = obstacle[2:]
        dispBA = pA - pB
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])
        if 2.2 * ROBOT_RADIUS > distBA:
            distBA = 2.2*ROBOT_RADIUS
        phi_obst = np.arcsin(2.2*ROBOT_RADIUS/distBA)
        phi_left = thetaBA + phi_obst
        phi_right = thetaBA - phi_obst

        # VO
        translation = vB
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i*2, :] = Atemp
        bvec[i*2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i*2 + 1, :] = Atemp
        bvec[i*2 + 1] = btemp

    # Create search-space
    th = np.linspace(0, 2*np.pi, 20)
    vel = np.linspace(0, VMAX, 5)

    vv, thth = np.meshgrid(vel, th)

    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()

    v_sample = np.stack((vx_sample, vy_sample))

    v_satisfying_constraints = check_constraints(v_sample, Amat, bvec)

    # Objective function
    size = np.shape(v_satisfying_constraints)[1]
    diffs = v_satisfying_constraints - \
        (np.dot((v_desired).reshape(2, 1),np.ones(size).reshape(1, size)))
    norm = np.linalg.norm(diffs, axis=0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_constraints[:, min_index])

    return cmd_vel


def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]

    for i in range(int(length/2)):
        v_sample = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

    return v_sample


def check_inside(v, Amat, bvec):
    v_out = []
    for i in range(np.shape(v)[1]):
        if not ((np.dot(Amat, v[:, i]) < bvec).all()):
            v_out.append(v[:, i])
    return np.array(v_out).T


def create_constraints(translation, angle, side):
    # create line
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)])
    line = np.cross(origin, point)
    line = translate_line(line, translation)

    if side == "left":
        line *= -1

    A = line[:2]
    b = -line[2]

    return A, b


def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2]
    return np.dot(matrix, line)


def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    return new_state


def callback(data):
    global start,goal,robot_state,robot_state_history,i,obstacle_to_be_passed,c

    # Convert the flattened array back to the original shape
    # array = np.array(data.data).reshape((4, 3))
    #
    # # Perform some operation on the array
    # result = np.mean(array)
    #
    # # Print the result
    # rospy.loginfo("Received array:\n%s", array)
    # rospy.loginfo("Result: %.2f", result)

    # obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)
    #
    # start = np.array([2, 5, 0, 0])
    # goal = np.array([10, 0, 0, 0])
    # #the above arrays are (x,y,_,_)
    #
    # robot_state = start
    # robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS)) #4 X NUMBER_OF_TIMESTEPS uninitialized matrix

    # for i in range(NUMBER_OF_TIMESTEPS):
    v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX) #returns normalized velocity vector
    array_rec = np.array(data.data).reshape((4, 2))
    obstacle_to_be_passed[:,i,:]=array_rec
    # print(obstacle_to_be_passed)
        # print("Obstacle is: ")
        # print(obstacles[:, i, :])
        # print(v_desired)
    control_vel = compute_velocity(
        robot_state, array_rec, v_desired)
        # print("Control vel is: ")
        # print(control_vel)
    robot_state = update_state(robot_state, control_vel)
        # print("Robot is: ")
        # print(robot_state)
        # arr_to_be_passed = np.column_stack((obstacles[:, i, :], robot_state))
        # print(arr_to_be_passed)
        # str_to_be_passed=np.array_str(arr_to_be_passed)
        # print(str_to_be_passed)
        # print(type(arr_to_be_passed[0][0]))
        # msg.robot_name.data=str_to_be_passed
        # msg.data=arr_to_be_passed.flatten().tolist()
    robot_state_history[:4, i] = robot_state
    i=i+1
    if(i==50 and c==0):
        plot_robot_and_obstacles(robot_state_history, obstacle_to_be_passed, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)
        print(obstacle_to_be_passed)
        c=1
    if(c==1):
        i=0
    rospy.sleep(0.1)

def talker():

    # pub=rospy.Publisher('message_client1',pp1,queue_size=1000)
    rospy.init_node('numpy_subscriber_node', anonymous=True)
    rospy.Subscriber('numpy_array_topic', Float64MultiArray, callback)
    # pub = rospy.Publisher('numpy_array_topic', Float64MultiArray, queue_size=10)
    # rospy.Subscriber('message_client2',custom_2,sub2)
    # rospy.Subscriber('message_client3',custom_3,sub3)
    # rospy.Subscriber('mal_bot_flag',malicious_flags,sub_flag)
    # rospy.init_node('custom_talker_1',anonymous=True)
    #
    rospy.spin()

    # r=rospy.Rate(1)
    # #
    # # msg=pp1()
    # # msg2=block1()
    # #
    # #
    # # msg.robot_name.data="Robot_1"
    # msg = Float64MultiArray()
    #
    # # pub.publish(msg)
    # #
    # # msg.location.x=2.0
    # # msg.location.y=3.0
    # # msg.location.theta=90.0
    # #
    # while not rospy.is_shutdown():
    #     obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)
    #
    #     start = np.array([5, 0, 0, 0])
    #     goal = np.array([5, 10, 0, 0])
    #     #the above arrays are (x,y,_,_)
    #
    #     robot_state = start
    #     robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS)) #4 X NUMBER_OF_TIMESTEPS uninitialized matrix
    #     print(type(obstacles[:, 0, :]))
    #     out_arr = np.array_str(obstacles[:, 0, :])
    #     # msg.robot_name.data=out_arr
    #
    #     for i in range(NUMBER_OF_TIMESTEPS):
    #         v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX) #returns normalized velocity vector
    #         # print("Obstacle is: ")
    #         # print(obstacles[:, i, :])
    #         # print(v_desired)
    #         control_vel = compute_velocity(
    #             robot_state, obstacles[:, i, :], v_desired)
    #         # print("Control vel is: ")
    #         # print(control_vel)
    #         robot_state = update_state(robot_state, control_vel)
    #         # print("Robot is: ")
    #         # print(robot_state)
    #         arr_to_be_passed = np.column_stack((obstacles[:, i, :], robot_state))
    #         # print(arr_to_be_passed)
    #         str_to_be_passed=np.array_str(arr_to_be_passed)
    #         # print(str_to_be_passed)
    #         # print(type(arr_to_be_passed[0][0]))
    #         # msg.robot_name.data=str_to_be_passed
    #         msg.data=arr_to_be_passed.flatten().tolist()
    #         robot_state_history[:4, i] = robot_state
    #         pub.publish(msg)
    #
    #     plot_robot_and_obstacles(
    #         robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)
    # #     rospy.loginfo("robot1")
    # #     rospy.loginfo(msg)
    #     # pub.publish(msg)
    # #     t1="x="+str(msg.location.x)+","+"y="+str(msg.location.y)+","+"w="+str(msg.location.theta)
    # #     # rospy.loginfo(t1)
    # #     # rospy.loginfo(t2)
    # #     # rospy.loginfo(t3)
    # #     hash1=create_hash_from_transaction([str(t1),str(t2),str(t3)])
    # #
    # #     rospy.loginfo("hash=")
    # #     rospy.loginfo(hash1)
    # #     msg2.robot_hash.data=hash1
    # #     pub_hash.publish(msg2)
    # #
    #     r.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
