import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot(x_history, u_history, t_history):
    # Plotting position history
    plt.figure(figsize=(12, 8))

    # Plotting position history
    plt.subplot(3, 1, 1)
    plt.plot(t_history, x_history[:, 0], label='x position')
    plt.plot(t_history, x_history[:, 1], label='y position')
    plt.plot(t_history, x_history[:, 2], label='z position')
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.legend()
    plt.title('Position History')

    # Plotting velocity history
    plt.subplot(3, 1, 2)
    plt.plot(t_history, x_history[:, 3], label='roll')
    plt.plot(t_history, x_history[:, 4], label='pitch')
    plt.plot(t_history, x_history[:, 5], label='yaw')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.legend()
    plt.title('Velocity History')

    plt.subplot(3, 1, 3)
    # plt.plot(t_history, u_history[:, 0], label='Thrust 1')
    # plt.plot(t_history, u_history[:, 1], label='Thrust 2')
    # plt.plot(t_history, u_history[:, 2], label='Thrust 3')
    # plt.plot(t_history, u_history[:, 3],   label='Thrust 4')

    plt.step(t_history, u_history[:, 0], label='Thrust 1')
    plt.step(t_history, u_history[:, 1], label='Thrust 2')
    plt.step(t_history, u_history[:, 2], label='Thrust 3')
    plt.step(t_history, u_history[:, 3],   label='Thrust 4')

    plt.xlabel('Time')
    plt.ylabel('Control Inputs')
    plt.legend()
    plt.title('Control Inputs History')

    plt.tight_layout()
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_history[:, 0], x_history[:, 1], x_history[:, 2], label='Quadrotor Path', color='b')

    plt.show()