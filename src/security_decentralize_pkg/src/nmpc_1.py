#!/usr/bin/env python

import rospy
from security_decentralize_pkg.msg import pp1
# from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray

from multi_robot_plot import plot_robot_and_obstacles
from create_obstacles import create_obstacles
import numpy as np
from scipy.optimize import minimize, Bounds
import time

SIM_TIME = 8.
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.5
VMAX = 2
VMIN = 0.2

# collision cost parameters
Qc = 5.
kappa = 4.

# nmpc parameters
HORIZON_LENGTH = int(4)
NMPC_TIMESTEP = 0.3
upper_bound = [(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2
lower_bound = [-(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2


def simulate(filename):
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

    start = np.array([5, 5])
    p_desired = np.array([5, 5])

    robot_state = start
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))

    for i in range(NUMBER_OF_TIMESTEPS):
        # predict the obstacles' position in future
        obstacle_predictions = predict_obstacle_positions(obstacles[:, i, :])
        xref = compute_xref(robot_state, p_desired,
                            HORIZON_LENGTH, NMPC_TIMESTEP)
        # compute velocity using nmpc
        vel, velocity_profile = compute_velocity(
            robot_state, obstacle_predictions, xref)
        robot_state = update_state(robot_state, vel, TIMESTEP)
        robot_state_history[:2, i] = robot_state

    plot_robot_and_obstacles(
        robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)


def compute_velocity(robot_state, obstacle_predictions, xref):
    """
    Computes control velocity of the copter
    """
    # u0 = np.array([0] * 2 * HORIZON_LENGTH)
    u0 = np.random.rand(2*HORIZON_LENGTH)
    def cost_fn(u): return total_cost(
        u, robot_state, obstacle_predictions, xref)

    bounds = Bounds(lower_bound, upper_bound)

    res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds)
    velocity = res.x[:2]
    return velocity, res.x

def compute_xref(start, goal, number_of_steps, timestep):
    dir_vec = (goal - start)
    norm = np.linalg.norm(dir_vec)
    if norm < 0.1:
        new_goal = start
    else:
        dir_vec = dir_vec / norm
        new_goal = start + dir_vec * VMAX * timestep * number_of_steps
    return np.linspace(start, new_goal, number_of_steps).reshape((2*number_of_steps))


def total_cost(u, robot_state, obstacle_predictions, xref):
    x_robot = update_state(robot_state, u, NMPC_TIMESTEP)
    c1 = tracking_cost(x_robot, xref)
    c2 = total_collision_cost(x_robot, obstacle_predictions)
    total = c1 + c2
    return total


def tracking_cost(x, xref):
    return np.linalg.norm(x-xref)


def total_collision_cost(robot, obstacles):
    total_cost = 0
    for i in range(HORIZON_LENGTH):
        for j in range(len(obstacles)):
            obstacle = obstacles[j]
            rob = robot[2 * i: 2 * i + 2]
            obs = obstacle[2 * i: 2 * i + 2]
            total_cost += collision_cost(rob, obs)
    return total_cost


def collision_cost(x0, x1):
    """
    Cost of collision between two robot_state
    """
    d = np.linalg.norm(x0 - x1)
    cost = Qc / (1 + np.exp(kappa * (d - 2*ROBOT_RADIUS)))
    return cost


def predict_obstacle_positions(obstacles):
    obstacle_predictions = []
    for i in range(np.shape(obstacles)[1]):
        obstacle = obstacles[:, i]
        obstacle_position = obstacle[:2]
        obstacle_vel = obstacle[2:]
        u = np.dot(np.vstack([np.eye(2)] * HORIZON_LENGTH) , obstacle_vel)
        obstacle_prediction = update_state(obstacle_position, u, NMPC_TIMESTEP)
        obstacle_predictions.append(obstacle_prediction)
    return obstacle_predictions

def update_state(x0, u, timestep):
    """
    Computes the states of the system after applying a sequence of control signals u on
    initial state x0
    """
    N = int(len(u) / 2)
    lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
    kron = np.kron(lower_triangular_ones_matrix, np.eye(2))

    new_state = np.dot(np.vstack([np.eye(2)] * int(N)) , x0)+ np.dot(kron , u * timestep)

    return new_state


def talker():
    c=0

    # pub=rospy.Publisher('message_client1',pp1,queue_size=1000)
    rospy.init_node('numpy_publisher_node', anonymous=True)
    pub = rospy.Publisher('numpy_array_topic', Float64MultiArray, queue_size=10)
    # rospy.Subscriber('message_client2',custom_2,sub2)
    # rospy.Subscriber('message_client3',custom_3,sub3)
    # rospy.Subscriber('mal_bot_flag',malicious_flags,sub_flag)
    # rospy.init_node('custom_talker_1',anonymous=True)
    #
    r=rospy.Rate(1)
    #
    # msg=pp1()
    # msg2=block1()
    #
    #
    # msg.robot_name.data="Robot_1"
    msg = Float64MultiArray()

    # pub.publish(msg)
    #
    # msg.location.x=2.0
    # msg.location.y=3.0
    # msg.location.theta=90.0
    array_to_be_passed = np.zeros((4,NUMBER_OF_TIMESTEPS,2))
    #
    while not rospy.is_shutdown():

        obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

        start = np.array([5, 0])
        p_desired = np.array([2, 10])

        robot_state = start
        robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))

        print(type(obstacles[:, 0, :]))
        out_arr = np.array_str(obstacles[:, 0, :])
        # print(obstacles)
        # print(type(obstacles))
        print(np.shape(obstacles[:, 0, :]))
        # msg.robot_name.data=out_arr

        for i in range(NUMBER_OF_TIMESTEPS):
            # predict the obstacles' position in future
            obstacle_predictions = predict_obstacle_positions(obstacles[:, i, :])
            xref = compute_xref(robot_state, p_desired,
                                HORIZON_LENGTH, NMPC_TIMESTEP)
            # compute velocity using nmpc
            vel, velocity_profile = compute_velocity(
                robot_state, obstacle_predictions, xref)
            robot_state = update_state(robot_state, vel, TIMESTEP)
            robot_state_history[:2, i] = robot_state

            # v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX) #returns normalized velocity vector
            # # print("Obstacle is: ")
            # print(obstacles[:, i, :])
            # # print(v_desired)
            # control_vel = compute_velocity(
            #     robot_state, obstacles[:, i, :], v_desired)
            # # print("Control vel is: ")
            # # print(control_vel)
            # robot_state = update_state(robot_state, control_vel)
            # # print("Robot is: ")
            # print(type(robot_state))
            # print(type(vel))
            new_rob_state=np.concatenate((robot_state,vel),axis=None)
            # print(new_rob_state)
            arr_to_be_passed = np.column_stack((obstacles[:, i, :], new_rob_state))
            array_to_be_passed[:,i,:]=arr_to_be_passed
            # print(arr_to_be_passed)
            str_to_be_passed=np.array_str(arr_to_be_passed)
            # print(str_to_be_passed)
            # print(type(arr_to_be_passed[0][0]))
            # msg.robot_name.data=str_to_be_passed
            msg.data=arr_to_be_passed.flatten().tolist()
            # robot_state_history[:4, i] = robot_state
            rospy.sleep(0.1)
            pub.publish(msg)

        # print(array_to_be_passed)
        # plot_robot_and_obstacles(robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)
        if(c==0):
            plot_robot_and_obstacles(robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)
            c=1
    #     rospy.loginfo("robot1")
    #     rospy.loginfo(msg)
        # pub.publish(msg)
    #     t1="x="+str(msg.location.x)+","+"y="+str(msg.location.y)+","+"w="+str(msg.location.theta)
    #     # rospy.loginfo(t1)
    #     # rospy.loginfo(t2)
    #     # rospy.loginfo(t3)
    #     hash1=create_hash_from_transaction([str(t1),str(t2),str(t3)])
    #
    #     rospy.loginfo("hash=")
    #     rospy.loginfo(hash1)
    #     msg2.robot_hash.data=hash1
    #     pub_hash.publish(msg2)
    #
        r.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
