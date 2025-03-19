from environments.environment3 import Environment as Env
from robots import Robot, StickRobot

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description="Simulate robot movement in a gradient-based environment.")
    parser.add_argument("--robot_type", "-r", type=int, default=2, choices=[1, 2], help="Type of robot (1 or 2).")
    parser.add_argument("--env_limit", "-l", type=int, default=10, help="Limit of the environment.")
    parser.add_argument("--obstacle_number", "-o", type=int, default=4, help="Number of obstacles in the environment.")
    parser.add_argument("--sigma", "-s", type=float, default=5, help="Sigma value for the environment.")
    return parser.parse_args()

def main():
    args = parse_arguments()
    robot_type = args.robot_type
    env_limit = args.env_limit
    obstacle_number = args.obstacle_number
    sigma = args.sigma

    # Initialize the environment
    env = Env([1 for _ in range(obstacle_number)], sigma=sigma, env_limit=env_limit)
    # Spawn a robot at random position
    x, y = 0, 0
    while True:
        x = np.random.uniform(-env_limit//2, env_limit//2)
        y = np.random.uniform(-env_limit//2, env_limit//2)
        if not env.is_collision(x, y, 3):
            break

    robot_length = 1.5
    if robot_type == 1:
        robot = Robot(x, y)
        path = [[robot.x, robot.y]]
        input_hist = [[0, 0]]
    else:
        theta = np.random.uniform(0, np.pi/2)
        robot = StickRobot(x, y, theta, robot_length)
        path = [[robot.x, robot.y, robot.theta]]
        input_hist = [[0, 0, 0]]
    
    # Simulate the robot movement
    dt = 0.05
    grad_hist = [[0, 0]]
    for _ in range(1500):
        if robot_type == 1:
            x, y = robot.bb_grad_descent(env)
            robot.apf_update(dt, env, goal=[x, y], k=0.5, k_rep=1.5, gamma=2, rho=2, eta_0=5, nu=0.8)
            path.append([robot.x, robot.y])
            grad_hist.append(robot.prev_dh)
            input_hist.append([robot.u_x, robot.u_y])
        else:
            x, y = robot.bb_grad_descent(env)
            robot.apf_update(dt, env, goal=[x, y], k=0.5, k_rep=0.6, gamma=2, rho=2, eta_0=3, nu=0.8, mul_theta=2.5, tail_orientation=0)
            path.append([robot.x, robot.y, robot.theta])    
            grad_hist.append(robot.prev_dh)
            input_hist.append([robot.u_x, robot.u_y, robot.u_theta]) 
        if robot.prev_dh is not None:
            if np.sqrt(robot.prev_dh[0]**2 + robot.prev_dh[1]**2) < 1e-2:
                break

    # Plot the robot path using matplotlib
    path = np.array(path)
    grad_hist = np.array(grad_hist)
    input_hist = np.array(input_hist)

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[3, 1])

    ax1 = fig.add_subplot(gs[:, 0])  # Main animation plot
    ax2 = fig.add_subplot(gs[0, 1])  # Inputs subplot
    ax3 = fig.add_subplot(gs[1, 1])  # Path subplot

    # Plot the environment data using matplotlib
    x = np.linspace(-env_limit, env_limit, 100)
    y = np.linspace(-env_limit, env_limit, 100)
    X, Y = np.meshgrid(x, y)
    Z = np.vectorize(env.h)(X, Y)
    contourf = ax1.contourf(X, Y, Z, levels=20, cmap='viridis')
    fig.colorbar(contourf, ax=ax1, orientation='vertical', label='H(x, y)')

    for diameter, x, y in env.obstacles:
        circle = plt.Circle((x, y), diameter / 2, color='blue', fill=True)
        ax1.add_patch(circle)

    # Add a line to represent the path
    path_line, = ax1.plot([], [], 'r--', alpha=0.25)

    # Initialize input and path subplots
    input_line_x, = ax2.plot([], [], 'b-', label='u_x', alpha=0.5)
    input_line_y, = ax2.plot([], [], 'r-', label='u_y', alpha=0.5)
    grad_line_x, = ax2.plot([], [], 'c-', label='grad_x', alpha=0.5)
    grad_line_y, = ax2.plot([], [], 'm-', label='grad_y', alpha=0.5)
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

    def update_r1(num, path, robot_circle, grad_arrows, input_arrows, path_line, input_line_x, input_line_y, grad_line_x, grad_line_y, path_line_x, path_line_y):
        robot_circle.center = (path[num, 0], path[num, 1])
        grad_arrows.set_offsets([path[num, 0], path[num, 1]])
        grad_arrows.set_UVC(grad_hist[num, 0], grad_hist[num, 1])
        input_arrows.set_offsets([path[num, 0], path[num, 1]])
        input_arrows.set_UVC(input_hist[num, 0], input_hist[num, 1])
        path_line.set_data(path[:num+1, 0], path[:num+1, 1])
        input_line_x.set_data(range(num+1), input_hist[:num+1, 0])
        input_line_y.set_data(range(num+1), input_hist[:num+1, 1])
        grad_line_x.set_data(range(num+1), grad_hist[:num+1, 0])
        grad_line_y.set_data(range(num+1), grad_hist[:num+1, 1])
        path_line_x.set_data(range(num+1), path[:num+1, 0])
        path_line_y.set_data(range(num+1), path[:num+1, 1])
        return grad_arrows, input_arrows, robot_circle, path_line, input_line_x, input_line_y, grad_line_x, grad_line_y, path_line_x, path_line_y

    def update_r2(num, path, stick_robot, robot_circle, grad_arrows, input_arrows, path_line, input_line_x, input_line_y, grad_line_x, grad_line_y, path_line_x, path_line_y, input_line_theta, path_line_theta):
        stick_robot.set_data([path[num, 0] - robot.length*np.cos(path[num, 2]), path[num, 0]], 
                             [path[num, 1] - robot.length*np.sin(path[num, 2]), path[num, 1]])
        robot_circle.center = (path[num, 0], path[num, 1])
        grad_arrows.set_offsets([path[num, 0], path[num, 1]])
        grad_arrows.set_UVC(grad_hist[num, 0], grad_hist[num, 1])
        input_arrows.set_offsets([path[num, 0], path[num, 1]])
        input_arrows.set_UVC(input_hist[num, 0], input_hist[num, 1])
        path_line.set_data(path[:num+1, 0], path[:num+1, 1])
        input_line_x.set_data(range(num+1), input_hist[:num+1, 0])
        input_line_y.set_data(range(num+1), input_hist[:num+1, 1])
        grad_line_x.set_data(range(num+1), grad_hist[:num+1, 0])
        grad_line_y.set_data(range(num+1), grad_hist[:num+1, 1])
        input_line_theta.set_data(range(num+1), input_hist[:num+1, 2])
        path_line_x.set_data(range(num+1), path[:num+1, 0])
        path_line_y.set_data(range(num+1), path[:num+1, 1])
        path_line_theta.set_data(range(num+1), path[:num+1, 2])
        return grad_arrows, input_arrows, stick_robot, robot_circle, path_line, input_line_x, input_line_y, grad_line_x, grad_line_y, path_line_x, path_line_y, input_line_theta, path_line_theta

    if robot_type == 1:
        robot_circle = plt.Circle((robot.x, robot.y), robot.diameter/2, color='orange', fill=True, zorder=2)
        ax1.add_patch(robot_circle)
        grad_arrows = ax1.quiver(path[0, 0], path[0, 1], grad_hist[0, 0], grad_hist[0, 1], color='red', scale=0.05, scale_units='xy', alpha=0.7, zorder=3)
        input_arrows = ax1.quiver(path[0, 0], path[0, 1], input_hist[0, 0], input_hist[0, 1], color='green', scale=0.1, scale_units='xy', alpha=0.5, zorder=3)
        ani = animation.FuncAnimation(fig, update_r1, frames=len(path), fargs=(path, robot_circle, grad_arrows, input_arrows, path_line, input_line_x, input_line_y, grad_line_x, grad_line_y, path_line_x, path_line_y), interval=dt*100, blit=True, repeat_delay=3000)
    else:
        stick_robot = plt.Line2D([robot.x - robot.length*np.cos(robot.theta), robot.x], 
                                 [robot.y - robot.length*np.sin(robot.theta), robot.y], color='orange', zorder=2)
        ax1.add_line(stick_robot)
        robot_circle = plt.Circle((robot.x, robot.y), 0.1, color='red', fill=True, zorder=2)
        ax1.add_patch(robot_circle)
        grad_arrows = ax1.quiver(path[0, 0], path[0, 1], grad_hist[0, 0], grad_hist[0, 1], color='red', scale=0.05, scale_units='xy', alpha=0.7, zorder=3)
        input_arrows = ax1.quiver(path[0, 0], path[0, 1], input_hist[0, 0], input_hist[0, 1], color='green', scale=0.1, scale_units='xy', alpha=0.5, zorder=3)
        ani = animation.FuncAnimation(fig, update_r2, frames=len(path), fargs=(path, stick_robot, robot_circle, grad_arrows, input_arrows, path_line, input_line_x, input_line_y, grad_line_x, grad_line_y, path_line_x, path_line_y, input_line_theta, path_line_theta), interval=dt*100, blit=True, repeat_delay=3000)

    ax1.set_xlim(-env_limit, env_limit)
    ax1.set_ylim(-env_limit, env_limit)
    ax1.set_xlabel('X-axis')
    ax1.set_ylabel('Y-axis')
    ax1.set_title('Environment Visualization'+' - Robot Type R'+str(robot_type))
    ax1.grid(True)
    ax1.set_aspect('equal', adjustable='box')

    plt.show()

if __name__ == "__main__":
    main()