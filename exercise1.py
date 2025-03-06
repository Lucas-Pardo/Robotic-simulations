from environments.environment1 import Environment as Env1
from environments.environment2 import Environment as Env2
from robots import Robot, StickRobot

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Initialize the environment
environment_type = 2
robot_type = 2
obstacle_number = 4
env_limit = 20
if environment_type == 1:
    env = Env1([1 for _ in range(obstacle_number)], env_limit)
    # Spawn a robot at random position
    x, y = 0, 0
    while True:
        x = np.random.uniform(-env_limit, env_limit)
        y = np.random.uniform(-env_limit, env_limit)
        if not env.is_collision(x, y, 2):
            break
else:
    env = Env2()
    env_limit = 5
    x, y = env.generate_start_position(1.1)

robot_length = 1.5
if robot_type == 1:
    robot = Robot(x, y)
    path = [[robot.x, robot.y]]
    input_hist = [[0, 0]]
else:
    if environment_type == 1:
        theta = np.random.uniform(0, np.pi/2)
    else:
        aux = np.arccos(min(abs(x - env.start_zone[0][0]), env.start_zone[0][1] - x) / robot_length)
        theta = np.random.uniform(aux + 0.05, np.pi - aux - 0.05)
    robot = StickRobot(x, y, theta, robot_length)
    path = [[robot.x, robot.y, robot.theta]]
    input_hist = [[0, 0, 0]]

# Simulate the robot movement
dt = 0.05 / environment_type
for _ in range(1500):
    if environment_type == 1:
        if robot_type == 1:
            robot.apf_update(dt, env, k=1, k_rep=1.8, gamma=2, rho=1.5, eta_0=6, nu=0.8)
            # robot.cbf_update(dt, env, alpha=0.6, beta=0.2)
            path.append([robot.x, robot.y])
            input_hist.append([robot.u_x, robot.u_y])
        else:
            robot.apf_update(dt, env, k=1, k_rep=1.2, gamma=2, rho=1.5, eta_0=5, nu=0.8, mul_theta=2.5, final_heading_k=0.6)
            # robot.cbf_update(dt, env, alpha=0.6, beta=0.2)
            path.append([robot.x, robot.y, robot.theta])
            input_hist.append([robot.u_x, robot.u_y, robot.u_theta])
        if np.sqrt(path[-1][0]**2 + path[-1][1]**2) < 5e-2:
            break
    else:
        if robot_type == 1:
            robot.apf_update(dt, env, k=0.6, k_rep=1.5, gamma=2, rho=1, eta_0=0.65, nu=0)
            path.append([robot.x, robot.y])
            input_hist.append([robot.u_x, robot.u_y])
        else:
            robot.apf_update(dt, env, k=0.6, k_rep=0.5, gamma=2, rho=1, eta_0=0.6, nu=0, mul_theta=1.2, tail_orientation=0, final_heading_k=0)
            path.append([robot.x, robot.y, robot.theta])
            input_hist.append([robot.u_x, robot.u_y, robot.u_theta])
        if np.abs(path[-1][0] - env.goal_position) < 1e-1:
            break

# Plot the robot path using matplotlib
path = np.array(path)
input_hist = np.array(input_hist)

fig = plt.figure(figsize=(14, 8))
gs = fig.add_gridspec(2, 2, width_ratios=[3, 1])

ax1 = fig.add_subplot(gs[:, 0])  # Main animation plot
ax2 = fig.add_subplot(gs[0, 1])  # Inputs subplot
ax3 = fig.add_subplot(gs[1, 1])  # Path subplot

# Plot the environment data using matplotlib
if environment_type == 1:
    for diameter, x, y in env.obstacles:
        circle = plt.Circle((x, y), diameter / 2, color='blue', fill=True)
        ax1.add_patch(circle)
else:
    for b_pos in env.boundaries:
        ax1.plot([b_pos[0][0], b_pos[1][0]], [b_pos[0][1], b_pos[1][1]], color='blue')
        circle = plt.Circle((env.goal_position, 0), 0.2, color='green', fill=True)
        ax1.add_patch(circle)

# Add a line to represent the path
path_line, = ax1.plot([], [], 'r--', alpha=0.25)

# Initialize input and path subplots
input_line_x, = ax2.plot([], [], 'b-', label='u_x', alpha=0.5)
input_line_y, = ax2.plot([], [], 'r-', label='u_y', alpha=0.5)
if robot_type == 2:
    input_line_theta, = ax2.plot([], [], 'g-', label='u_theta', alpha=0.5)
ax2.legend()
ax2.set_xlim(0, len(input_hist))
ax2.set_ylim(1.2*np.min(input_hist), 1.2*np.max(input_hist))
ax2.set_xlabel('Time step')
ax2.set_ylabel('Input')

path_line_x, = ax3.plot([], [], 'b-', label='x', alpha=0.5)
path_line_y, = ax3.plot([], [], 'r-', label='y', alpha=0.5)
if robot_type == 2:
    path_line_theta, = ax3.plot([], [], 'g-', label='theta', alpha=0.5)
ax3.legend()
ax3.set_xlim(0, len(path))
ax3.set_ylim(1.2*np.min(path), 1.2*np.max(path))
ax3.set_xlabel('Time step')
ax3.set_ylabel('Position')

def update_r1(num, path, robot_circle, input_arrows, path_line, input_line_x, input_line_y, path_line_x, path_line_y):
    robot_circle.center = (path[num, 0], path[num, 1])
    input_arrows.set_offsets([path[num, 0], path[num, 1]])
    input_arrows.set_UVC(input_hist[num, 0], input_hist[num, 1])
    path_line.set_data(path[:num+1, 0], path[:num+1, 1])
    input_line_x.set_data(range(num+1), input_hist[:num+1, 0])
    input_line_y.set_data(range(num+1), input_hist[:num+1, 1])
    path_line_x.set_data(range(num+1), path[:num+1, 0])
    path_line_y.set_data(range(num+1), path[:num+1, 1])
    return robot_circle, input_arrows, path_line, input_line_x, input_line_y, path_line_x, path_line_y

def update_r2(num, path, stick_robot, robot_circle, input_arrows, path_line, input_line_x, input_line_y, path_line_x, path_line_y, input_line_theta, path_line_theta):
    stick_robot.set_data([path[num, 0] - robot.length*np.cos(path[num, 2]), path[num, 0]], 
                         [path[num, 1] - robot.length*np.sin(path[num, 2]), path[num, 1]])
    robot_circle.center = (path[num, 0], path[num, 1])
    input_arrows.set_offsets([path[num, 0], path[num, 1]])
    input_arrows.set_UVC(input_hist[num, 0], input_hist[num, 1])
    path_line.set_data(path[:num+1, 0], path[:num+1, 1])
    input_line_x.set_data(range(num+1), input_hist[:num+1, 0])
    input_line_y.set_data(range(num+1), input_hist[:num+1, 1])
    input_line_theta.set_data(range(num+1), input_hist[:num+1, 2])
    path_line_x.set_data(range(num+1), path[:num+1, 0])
    path_line_y.set_data(range(num+1), path[:num+1, 1])
    path_line_theta.set_data(range(num+1), path[:num+1, 2])
    return stick_robot, robot_circle, input_arrows, path_line, input_line_x, input_line_y, path_line_x, path_line_y, input_line_theta, path_line_theta

# Animate the robot movement and graphs
if robot_type == 1:
    robot_circle = plt.Circle((robot.x, robot.y), robot.diameter/2, color='orange', fill=True)
    ax1.add_patch(robot_circle)
    input_arrows = ax1.quiver(path[0, 0], path[0, 1], input_hist[0, 0], input_hist[0, 1], color='green', scale=0.1*environment_type, scale_units='xy', alpha=0.5)
    ani = animation.FuncAnimation(fig, update_r1, frames=len(path), fargs=(path, robot_circle, input_arrows, path_line, input_line_x, input_line_y, path_line_x, path_line_y), interval=dt*100, blit=True, repeat_delay=3000)
else:
    stick_robot = plt.Line2D([robot.x - robot.length*np.cos(robot.theta), robot.x], 
                             [robot.y - robot.length*np.sin(robot.theta), robot.y], color='orange', alpha=1)
    ax1.add_line(stick_robot)
    robot_circle = plt.Circle((robot.x, robot.y), 0.1, color='red', fill=True)
    ax1.add_patch(robot_circle)
    input_arrows = ax1.quiver(path[0, 0], path[0, 1], input_hist[0, 0], input_hist[0, 1], color='green', scale=0.1*environment_type, scale_units='xy', alpha=0.5)
    ani = animation.FuncAnimation(fig, update_r2, frames=len(path), fargs=(path, stick_robot, robot_circle, input_arrows, path_line, input_line_x, input_line_y, path_line_x, path_line_y, input_line_theta, path_line_theta), interval=dt*100, blit=True, repeat_delay=3000)

ax1.set_xlim(-env_limit, env_limit)
ax1.set_ylim(-env_limit, env_limit)
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
ax1.set_title('Environment Visualization E'+str(environment_type)+' - Robot Type R'+str(robot_type))
ax1.grid(True)
ax1.set_aspect('equal', adjustable='box')

plt.show()