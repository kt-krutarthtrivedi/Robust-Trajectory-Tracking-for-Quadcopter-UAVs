#!/usr/bin/env python3
import numpy as np
import sympy as sym

class QuinticPolynomialTrajectoryGeneration():
    '''

    '''
    def __init__(self, waypoints, tf_vec) -> None:
        
        self.waypoints = waypoints
        self.no_of_waypoints = len(waypoints)
        self.tf_vec = tf_vec
        self.traj_coefficients = []
        

        self.t0 = 0
        
        self.q0_d  = 0
        self.q0_dd = 0
        self.qf_d  = 0
        self.qf_dd = 0

        t = sym.Symbol('t')
        self.t_vec    = np.array([1, t, t**2, t**3, t**4, t**5])
        self.t_d_vec  = np.array([0, 1, 2*t, 3*(t**2), 4*(t**3), 5*(t**4)])
        self.t_dd_vec = np.array([0, 0, 2, 6*t, 12*(t**2), 20*(t**3)])

        self.q    = []
        self.q_d  = []
        self.q_dd = []

    def generateTrajectoryCoefficients(self):

        for j in range(len(self.waypoints[0])):
            for i in range(1, self.no_of_waypoints):
                t0 = self.t0
                tf = self.tf_vec[i-1]

                q0 = self.waypoints[i-1][j]
                qf = self.waypoints[i][j]

            
                q0_d  = self.q0_d
                q0_dd = self.q0_dd
                qf_d  = self.qf_d
                qf_dd = self.qf_dd
            

                A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5], 
                        [0, 1, 2*t0, 3*(t0**2), 4*(t0**3), 5*(t0**4)], 
                        [0, 0, 2, 6*t0, 12*(t0**2), 20*(t0**3)], 
                        [1, tf, tf**2, tf**3, tf**4, tf**5],
                        [0, 1, 2*tf, 3*(tf**2), 4*(tf**3), 5*(tf**4)], 
                        [0, 0, 2, 6*tf, 12*(tf**2), 20*(tf**3)]])
            
                b = np.array([q0, q0_d, q0_dd, qf, qf_d, qf_dd])

                a = np.matmul(np.linalg.inv(A) , b)
                
                self.traj_coefficients.append(a)

        self.traj_coefficients = np.array(self.traj_coefficients)   
        self.traj_coefficients = np.resize(self.traj_coefficients, [3,5,6])   # Can remove the hardcoded value of resize.

        return self.traj_coefficients
    
    def generateTrajectory(self):
        '''
        '''
        self.generateTrajectoryCoefficients()
        xyz, no_of_waypoints, no_of_coefficients = self.traj_coefficients.shape

        for i in range(xyz):
            for j in range(no_of_waypoints):
                coefficient_vec = self.traj_coefficients[i][j][:]
                q    = coefficient_vec * self.t_vec
                q_d  = coefficient_vec * self.t_d_vec
                q_dd = coefficient_vec * self.t_dd_vec

                q    = np.sum(q)
                q_d  = np.sum(q_d)
                q_dd = np.sum(q_dd)
                
                self.q.append(q)
                self.q_d.append(q_d)
                self.q_dd.append(q_dd)
        
        self.q    = np.array(self.q)
        self.q_d  = np.array(self.q_d)
        self.q_dd = np.array(self.q_dd)

        self.q    = np.resize(self.q, [3,5])
        self.q_d  = np.resize(self.q_d, [3,5])
        self.q_dd = np.resize(self.q_dd, [3,5])

        return self.q, self.q_d, self.q_dd



# w = [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [0, 0, 1]]
# tf_ = [5, 15, 15, 15, 15]
# qpt = QuinticPolynomialTrajectoryGeneration(w, tf_)
# traj_q, traj_q_d, traj_q_dd = qpt.generateTrajectory()

# print(traj_q.shape)
# print(traj_q_d.shape)
# print(traj_q_dd.shape)