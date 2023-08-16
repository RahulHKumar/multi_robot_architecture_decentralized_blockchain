#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
import heapq

# Define the map with obstacles and goal positions
map = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

# Define the goal positions for each robot
goal_positions = [(4, 4), (2, 2), (1, 3)]

# Define the starting positions for each robot
start_positions = [(0, 0), (1, 0), (0, 1)]

# Define the movements: up, down, left, right
movements = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Define the number of robots
num_robots = len(goal_positions)

def heuristic(position, goal):
    # Manhattan distance heuristic
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])

def distribute_a_star(start_positions, goal_positions):
    # Initialize the priority queue
    queue = []

    # Initialize the closed set for each robot
    closed_sets = [[] for _ in range(num_robots)]

    # Initialize the cost and path dictionaries for each robot
    cost = [{} for _ in range(num_robots)]
    path = [{} for _ in range(num_robots)]

    # Add the starting positions to the priority queue
    for i in range(num_robots):
        heapq.heappush(queue, (0, start_positions[i], i))
        cost[i][start_positions[i]] = 0

    # Run the distributed A* algorithm
    while queue:
        _, current, current_robot = heapq.heappop(queue)

        # Check if the current position is the goal for the current robot
        if current == goal_positions[current_robot]:
            # Return the path for each robot
            paths = []
            for i in range(num_robots):
                path_i = []
                pos = goal_positions[i]
                while pos != start_positions[i]:
                    path_i.append(pos)
                    pos = path[i].get(str(pos), None)
                path_i.append(start_positions[i])
                paths.append(path_i[::-1])
            return paths

        # Expand the current position
        for move in movements:
            neighbor = (current[0] + move[0], current[1] + move[1])

            # Check if the neighbor is valid and not in the closed set of any other robot
            valid_neighbor = (
                0 <= neighbor[0] < len(map) and
                0 <= neighbor[1] < len(map[0]) and
                map[neighbor[0]][neighbor[1]] == 0
            )
            if not valid_neighbor or neighbor in closed_sets[current_robot]:
                continue

            # Calculate the cost to reach the neighbor
            neighbor_cost = cost[current_robot][current] + 1

            # Check if the neighbor has not been visited or has a lower cost
            if (
                neighbor not in cost[current_robot] or
                neighbor_cost < cost[current_robot][neighbor]
            ):
                cost[current_robot][neighbor] = neighbor_cost
                priority = neighbor_cost + heuristic(neighbor, goal_positions[current_robot])
                heapq.heappush(queue, (priority, neighbor, current_robot))
                path[current_robot][str(neighbor)] = current
                closed_sets[current_robot].append(neighbor)



# Run the distributed A* algorithm
paths = distribute_a_star(start_positions, goal_positions)
#
# # Set up the figure and axes
# fig, ax = plt.subplots()
# ax.set_xticks([])
# ax.set_yticks([])

# Set up the figure and axes
fig, ax = plt.subplots()
ax.set_xticks([])
ax.set_yticks([])
im = ax.imshow(map, cmap=cmap, origin='lower')

# Function to update the animation
def update(frame):
    ax.clear()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.imshow(map, cmap=cmap, origin='lower')

    # Plot the robots' paths
    for i in range(num_robots):
        if frame < len(paths[i]):
            x, y = paths[i][frame]
            ax.plot(y, x, 'bo', markersize=8)  # Blue circles represent robots
            ax.text(y, x, f'R{i+1}', color='w', ha='center', va='center', fontsize=12)  # Robot labels

    # Plot the goal positions
    for goal in goal_positions:
        x, y = goal
        ax.plot(y, x, 'go', markersize=8)  # Green circles represent goal positions

    # Plot the obstacles
    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] == 1:
                ax.plot(j, i, 's', color='gray', markersize=12)  # Gray squares represent obstacles))


# Create the animation
ani = animation.FuncAnimation(fig, update, frames=max(len(path) for path in paths), interval=1000, repeat=True)

# Show the animation
plt.show()

# Run the distributed A* algorithm
# paths = distribute_a_star(start_positions, goal_positions)
#
# # Create a colormap for visualization
# cmap = ListedColormap(['white', 'gray', 'green', 'red'])
#
# # Set up the figure and axes
# # fig, ax = plt.subplots()
#
# # Set up the figure and axes
# fig, ax = plt.subplots()
# ax.set_xticks([])
# ax.set_yticks([])
# im = ax.imshow(map, cmap=cmap, origin='lower')
# Create a directory to save the frames
# frames_dir = "animation_frames"
# os.makedirs(frames_dir, exist_ok=True)

# Update the plot for each frame
# for frame in range(max(len(path) for path in paths)):
#     update(frame)
#     plt.pause(1)  # Pause for 1 second between frames
#
# # Show the animation
# plt.show()
# # for frame in range(max(len(path) for path in paths)):
# #     update(frame)
# #
# # # Show a message after saving the frames
# # print(f"Frames saved in {frames_dir}/")

print("Finished")
