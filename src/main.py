"""
Project:        Robust Trajectory Tracking for Quadcopter UAVs
Description:    The objective of this project is to develop a robust control scheme to enable a quadrotor to track
                desired trajectories in the presence of external disturbances.The control design is tested on the Crazyflie 2.0 platform. 
                Crazyflie is a quadrotor classified as a micro air vehicle (MAV), as it only weighs 27 grams and can fit in a hand.
Contributor:    Krutarth Ambarish Trivedi (ktrivedi@wpi.edu)
"""

# !/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)

        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        
        self.xd_series = []
        self.xd_d_series = []
        self.xd_dd_series = []

        self.yd_series = []
        self.yd_d_series = []
        self.yd_dd_series = []

        self.zd_series = []
        self.zd_d_series = []
        self.zd_dd_series = []

        # Desired waypoints and respective time provided in the problem statement.
        self.waypoints = [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [0, 0, 1]]     # Waypoints 
        self.tf_vec = [0, 5, 15, 15, 15, 15]    # Expected time between two waypoints. (i.e., (0,0,0) --> (0,0,1) in 5 sec., (0,0,1) ---> (1,0,1) in 15 sec. and so on.) 
        self.tf_vec_ = np.cumsum(self.tf_vec)   # Cumulative time vector (i.e., [0, 5, 20, 35, 50, 65] for the given problem statement)
        self.isValidWaypoint = True

        # Set it True to use the sliding mode with boundary layer
        self.includeBoundaryLayer = True

        # Set the constant for boundary layer condition
        self.boundaryLayer = 0.01

        # Physical parameters of the quadcopter
        self.m = 27*(10**-3)
        self.l = 46*(10**-3)
        self.Ix = 16.57171*(10**-6)
        self.Iy = 16.57171*(10**-6)
        self.Iz = 29.261652*(10**-6)
        self.Ip = 12.65625 *(10**-8)
        self.kf = 1.28192*(10**-8)
        self.km = 5.964552*(10**-3)       
        self.g = 9.8
        self.w_min = 0
        self.w_max = 2618

        self.omega = 0

        # Allocation matrix ---> Maps the control inputs to rotor velocities
        self.ak = 1/(4*self.kf)
        self.bk = sqrt(2)*self.ak/self.l 
        self.alloc = np.array([[self.ak,   -self.bk,    -self.bk,    -self.ak/self.km],
                               [self.ak,   -self.bk,     self.bk,     self.ak/self.km],
                               [self.ak,    self.bk,     self.bk,    -self.ak/self.km],
                               [self.ak,    self.bk,    -self.bk,     self.ak/self.km]])
        
        # ------------- Tuning Parameters -----------------#
        # For robust control design of Fx and Fy.
        self.Kp = 30
        self.Kd = 5

        # [K_z  K_phi  K_theta   K_psi]
        self.K = [2, 140, 180, 5]

        # [Lambda_z Lambda_phi  Lambda_theta Lambda_psi]
        self.lam = [2 , 12.5, 10, 5]
        # --------------------------------------------------#

        # --------- Provided in Problem Statement --------- #
        # The desired Yaw Angle, and also the desired angular velocities  
        # and desired angular acceleration can be considerd zero.
        # Roll  ----> phi
        # Pitch ----> theta
        # yaw   ----> psi

        # Desired yaw angle
        self.psid = 0

        # Desired Angular Velocities
        self.phid_dot   = 0
        self.thetad_dot = 0
        self.psid_dot   = 0

        # Desired Angular Accelerations
        self.phid_ddot   = 0
        self.thetad_ddot = 0
        self.psid_ddot   = 0
        # --------------------------------------------------#
        rospy.on_shutdown(self.save_data)

    def traj_coeff(self, t0, tf, pi, pf):
        """Compute the coefficients for the desired trajectory

        NB: Given that velocity and acceleration at each Waypoint is 0 i.e pi_d = pi_dd = pf_d = pf_dd = 0

        Parameters
        ----------
        t0 : double 
            start time

        tf : double
            final time

        pi : int
            start point

        pf : int 
            final point

        Returns
        -------
        coeff : a numpy array
            calculated coefficients 
        """
        A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5], 
                    [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4,],
                    [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3,],
                    [1, tf, tf**2, tf**3, tf**4, tf**5],
                    [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                    [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
        
        b = np.array([pi, 0, 0, pf, 0, 0])

        coeff = np.linalg.solve(A,b)

        return coeff
    
    def traj_gen(self, coeff_x, coeff_y, coeff_z):
        """Generate the desired trajectories based on the given coefficients
        Note: Desired trajectories ---> Position, Velocity and Acceleration

        Parameters
        ----------
        coeff_x : a numpy array 
            calculated coefficients for desired trajectories in x-coordinates

        coeff_y : a numpy array 
            calculated coefficients for desired trajectories in y-coordinates

        coeff_z : a numpy array 
            calculated coefficients for desired trajectories in z-coordinates

        Returns
        -------
        xd : a numpy array
            calculated desired position in x-coordinates

        xd_dot : a numpy array
            calculated desired velocity in x-coordinates

        xd_ddot : a numpy array
            calculated desired acceleration in x-coordinates

        yd : a numpy array
            calculated desired position in y-coordinates

        yd_dot : a numpy array
            calculated desired velocity in y-coordinates

        yd_ddot : a numpy array
            calculated desired acceleration in y-coordinates

        zd : a numpy array
            calculated desired position in z-coordinates

        zd_dot : a numpy array
            calculated desired velocity in z-coordinates

        zd_ddot : a numpy array
            calculated desired acceleration in z-coordinates
        """
        T = np.transpose(np.array([[1, self.t, self.t**2, self.t**3, self.t**4, self.t**5],  
                                [0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4], 
                                [0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3]]))

        traj_x = coeff_x @ T
        xd = traj_x[0]
        xd_dot = traj_x[1]
        xd_ddot = traj_x[2]

        traj_y = coeff_y @ T
        yd = traj_y[0]
        yd_dot = traj_y[1]
        yd_ddot = traj_y[2]

        traj_z = coeff_z @ T
        zd = traj_z[0]
        zd_dot = traj_z[1]
        zd_ddot = traj_z[2]

        return xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot

    def traj_evaluate(self):
        """Evaluate the trajectory for the given time
        Note: It calculates the desired trajectories for
        the desired waypoints and their respective timings in real-time

        Parameters
        ----------
        None

        Returns
        -------
        The generated trajectories
            @see: self.traj_gen(self, coeff_x, coeff_y, coeff_z)
        """
        if self.t<= self.tf_vec_[1]:    #P0 (0,0,0) to P1 (0,0,1)
            idx = 0

        elif self.t<= self.tf_vec_[2]:  #P1 (0,0,1) to P2 (1,0,1)
            idx = 1

        elif self.t<= self.tf_vec_[3]:  #P2 (1,0,1) to P3 (1,1,1)
            idx = 2

        elif self.t<= self.tf_vec_[4]:  #P3 (1,1,1) to P4 (0,1,1)
            idx = 3

        elif self.t<= self.tf_vec_[5]:  #P4 (0,1,1) to P5 (0,0,1)
            idx = 4

        else: # No input after 65 secs
            coeff_x = np.zeros(6)
            coeff_y = np.zeros(6)
            coeff_z = np.zeros(6)
            self.isValidWaypoint = False

        if (self.isValidWaypoint):
            coeff_x = self.traj_coeff(self.tf_vec_[idx],self.tf_vec_[idx+1],
                                    self.waypoints[idx][0],self.waypoints[idx+1][0])
            coeff_y = self.traj_coeff(self.tf_vec_[idx],self.tf_vec_[idx+1],
                                    self.waypoints[idx][1],self.waypoints[idx+1][1])
            coeff_z = self.traj_coeff(self.tf_vec_[idx],self.tf_vec_[idx+1],
                                    self.waypoints[idx][2],self.waypoints[idx+1][2])
            
        return self.traj_gen(coeff_x, coeff_y, coeff_z)

    def wrap_angle(self, angle):
        """Wrap the given angle to -pi to pi

        Parameters
        ----------
        angle : rad
            an angle in rad

        Returns
        -------
        rad
            an angle wrapped between -pi to pi
        """
        return np.arctan2(np.sin(angle),np.cos(angle))
    
    def sign_function(self, S):
        """Decide the sign value for the given number

        Parameters
        ----------
        S : double
            a numeric value

        Returns
        -------
        sign : int
            a numeric value indicating the sign. Either 1 or -1.
        """
        if S >= 0:
            sign = +1
        else:
            sign = -1
        return sign
    
    def sat_function(self, S):
        """Decide the sat value for the given number

        NB: This function requires self.boundaryLayer to be tuned as per the desired results.

        Note: The saturation function is used to prevent the chattering in the control input
        Please refer to Lecture21 for more details.

        Parameters
        ----------
        S : double
            a numeric value

        Returns
        -------
        double
            a numeric value 
        """
        return min(max(S/self.boundaryLayer, -1), 1)

    def sign_sat_function(self, S):
        """Decide between sign_function and sat_function based on the requirements set using 
        self.includeBoundaryLayer and return the calculated value

        Parameters
        ----------
        S : double
            a numeric value

        Returns
        -------
        sign or sat : double
            a numeric value 
        """
        if(self.includeBoundaryLayer):
            return self.sat_function(S)
        else:
            return self.sign_function(S)
        
    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding trajectories
        xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot = self.traj_evaluate()

        if (self.mutex_lock_on is not True):
            self.xd_series.append(xd)
            self.yd_series.append(yd)
            self.zd_series.append(zd)

            self.xd_d_series.append(xd_dot)
            self.yd_d_series.append(yd_dot)
            self.zd_d_series.append(zd_dot)

            self.xd_dd_series.append(xd_ddot)
            self.yd_dd_series.append(yd_ddot)
            self.zd_dd_series.append(zd_ddot)
        
        # Current position received from Gazebo
        x = xyz[0,0]
        y = xyz[1,0]
        z = xyz[2,0]

        # Current velocity received from Gazebo
        x_dot = xyz_dot[0,0]
        y_dot = xyz_dot[1,0]
        z_dot = xyz_dot[2,0]

        # Current roll, pitch and yaw angles received from Gazebo
        phi = rpy[0,0]
        theta = rpy[1,0]
        psi = rpy[2,0]

        # Current roll, pitch and yaw angular velocities received from Gazebo
        phi_dot = rpy_dot[0,0]
        theta_dot = rpy_dot[1,0]
        psi_dot = rpy_dot[2,0]

        # The control law for u1 -> Z Coordinate
        u1 = self.m * (zd_ddot + self.g - self.lam[0] * (z_dot - zd_dot)  - self.K[0] * \
                    self.sign_sat_function((z_dot - zd_dot) + self.lam[0] * (z - zd))) * (1/(np.cos(phi)*np.cos(theta)))
        
        # Calculate the force in x and y direction using the robust control law
        Fx = self.m*(-self.Kp*(x-xd) - self.Kd*(x_dot-xd_dot) + xd_ddot)
        Fy = self.m*(-self.Kp*(y-yd) - self.Kd*(y_dot-yd_dot) + yd_ddot)
        theta_d = asin(max(min(Fx / u1,1),-1))
        phi_d   = asin(-max(min(Fy / u1,1),-1))

        # The control law for u2 -> Phi 
        u2 = self.Ix*((-theta_dot*psi_dot*((self.Iy-self.Iz)/self.Ix))+self.Ip*self.omega*theta_dot/self.Ix+ self.phid_ddot- self.lam[1]*(phi_dot - self.phid_dot)- \
                    self.K[1]*self.sign_sat_function(phi_dot - self.phid_dot + self.lam[1]*self.wrap_angle(phi-phi_d)))

        # The control law for u3 -> Theta
        u3 = self.Iy*((-phi_dot*psi_dot*((self.Iz-self.Ix)/self.Iy))-self.Ip*self.omega*phi_dot/self.Ix+self.thetad_ddot-self.lam[2]*(theta_dot-self.thetad_dot)- \
                    self.K[2]*self.sign_sat_function(theta_dot -self.thetad_dot + self.lam[2] *self.wrap_angle(theta-theta_d)))
         
        # The control law for u4 -> Psi
        u4 = self.Iz*(((-phi_dot*theta_dot*((self.Ix-self.Iy)/self.Iz))+self.psid_ddot-self.lam[3]*(psi_dot-self.psid_dot)- \
                    self.K[3]*self.sign_sat_function(psi_dot - self.psid_dot+ self.lam[3] * self.wrap_angle(psi- self.psid))))

        # The control input
        u = np.array([u1,u2,u3,u4])
    
        # Calculate the rotor velocities
        motor_vel = (self.alloc @ u)**(1/2)
        
        # Restrict the rotor velocities 
        motor_vel = np.clip(motor_vel, a_min = self.w_min, a_max = self.w_max)

        self.omega = motor_vel[0] - motor_vel[1] + motor_vel[2] - motor_vel[3]

        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]]
        self.motor_speed_pub.publish(motor_speed)
              
    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])], [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

    # save the actual and desired trajectory data
    def save_data(self):
        currentDirectory = os.getcwd()
        fileDirectory = os.path.join(currentDirectory, 'log.pkl')
        with open(fileDirectory,"wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series, 
                        self.xd_series, self.yd_series, self.zd_series, 
                        self.xd_d_series, self.yd_d_series, self.zd_d_series, 
                        self.xd_dd_series, self.yd_dd_series, self.zd_dd_series], fp)

if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()

    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")