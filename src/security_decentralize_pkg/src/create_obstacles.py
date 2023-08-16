import numpy as np

def create_obstacles(sim_time, num_timesteps):
    # Obstacle 1
    v = -1.4
    p0 = np.array([5, 12])
    obst = create_robot(p0, v, np.pi/2, sim_time,
                        num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = obst
    # # Obstacle 2
    # v = 2
    # p0 = np.array([0, 5])
    # obst = create_robot(p0, v, 0, sim_time, num_timesteps).reshape(
    #     4, num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    # # Obstacle 3
    # v = 2
    # p0 = np.array([10, 10])
    # obst = create_robot(p0, v, -np.pi * 3 / 4, sim_time, num_timesteps).reshape(4,
    #                                                                             num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))
    # # Obstacle 4
    # v = 2
    # p0 = np.array([7.5, 2.5])
    # obst = create_robot(p0, v, np.pi * 3 / 4, sim_time, num_timesteps).reshape(4,
    #                                                                            num_timesteps, 1)
    # obstacles = np.dstack((obstacles, obst))

    #obstacle will have an array for position of each robot at a particular time instance
    # print(obstacles)

    return obstacles


def create_robot(p0, v, theta, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    #straight line for some fixed number of steps at const velocity
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    #p is a stack of all points that will be taken by the robot
    return p
