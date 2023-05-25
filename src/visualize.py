#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import Axes3D

def visualization(x_series, y_series, z_series):
    # load csv file and plot trajectory
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # Data for a three-dimensional line
    ax.plot3D(x_series, y_series, z_series, 'blue')
    ax.plot3D([0, 0, 1, 1, 0, 0], [0, 0, 0, 1, 1, 0], [0, 1, 1, 1, 1, 1], 'green')
    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.minorticks_on()
    plt.grid(which='both')
    plt.title("Trajectory")
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.savefig('trajectory.png', dpi = 300)
    plt.show()

def desired_trajectory_visualization(t_series, xd_series, yd_series, zd_series, 
                                    xd_d_series, yd_d_series, zd_d_series, 
                                    xd_dd_series, yd_dd_series, zd_dd_series):
    # Plot X-Desired Trajectories and save the figure
    fig = plt.figure()

    plt.subplot(3,1,1)
    plt.plot(t_series, xd_series)
    plt.xlabel('t (s)')
    plt.ylabel('x (m)')
    plt.title("X Desired Trajectories")

    plt.subplot(3,1,2)
    plt.plot(t_series, xd_d_series)
    plt.xlabel('t (s)')
    plt.ylabel('x_vel (m/s)')

    plt.subplot(3,1,3)
    plt.plot(t_series, xd_dd_series)
    plt.xlabel('t (s)')
    plt.ylabel('x_accel (m/s^2)')
    
    plt.savefig('x_trajectory.png', dpi = 300)
    plt.show()

    # Plot Y-Desired Trajectories and save the figure
    fig = plt.figure()

    plt.subplot(3,1,1)
    plt.plot(t_series, yd_series)
    plt.xlabel('t (s)')
    plt.ylabel('y (m)')
    plt.title("Y Desired Trajectories")

    plt.subplot(3,1,2)
    plt.plot(t_series, yd_d_series)
    plt.xlabel('t (s)')
    plt.ylabel('y_vel (m/s)')

    plt.subplot(3,1,3)
    plt.plot(t_series, yd_dd_series)
    plt.xlabel('t (s)')
    plt.ylabel('y_accel (m/s^2)')
    
    plt.savefig('y_trajectory.png', dpi = 300)
    plt.show()

    # Plot Z-Desired Trajectories and save the figure
    fig = plt.figure()

    plt.subplot(3,1,1)
    plt.plot(t_series, zd_series)
    plt.xlabel('t (s)')
    plt.ylabel('z (m)')
    plt.title("Z Desired Trajectories")
    plt.subplot(3,1,2)
    plt.plot(t_series, zd_d_series)
    plt.xlabel('t (s)')
    plt.ylabel('z_vel (m/s)')
    plt.subplot(3,1,3)
    plt.plot(t_series, zd_dd_series)
    plt.xlabel('t (s)')
    plt.ylabel('z_accel (m/s^2)')
    
    plt.savefig('z_trajectory.png', dpi = 300)
    plt.show()


if __name__ == '__main__':
    file = open("log.pkl",'rb')
    
    t_series, x_series, y_series, z_series, \
    xd_series, yd_series, zd_series, \
    xd_d_series, yd_d_series, zd_d_series, \
    xd_dd_series, yd_dd_series, zd_dd_series = pickle.load(file)

    file.close()

    visualization(x_series, y_series, z_series)
    desired_trajectory_visualization(t_series, xd_series, yd_series, zd_series, 
                                    xd_d_series, yd_d_series, zd_d_series, 
                                    xd_dd_series, yd_dd_series, zd_dd_series)