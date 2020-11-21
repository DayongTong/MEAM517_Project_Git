# this file construct a spacecraft object that can compute the optimal feedback controller

# TESTING SYNC ATTENSION PLZZZZ


import numpy as np
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos, tan
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp

class Spacecraft(object):
#   '''
#   Constructor. Compute function S(t) using S(t) = L(t) L(t)^t, by integrating backwards
#   from S(tf) = Qf. We will then use S(t) to compute the optimal controller efforts in 
#   the compute_feedback() function
#   '''
    def __init__(self, Q, R, Qf, tf,x_d,u_d):

        self.x_d = x_d
        self.u_d = u_d
        self.Q = Q
        self.R = R
        self.g = 3.71         
        self.Isp = 302.39


        # ''' 
        # We are integrating backwards from Qf
        # '''

        # Get L(tf) L(tf).T = S(tf) by decomposing S(tf) using Cholesky decomposition
        L0 = cholesky(Qf).transpose()

        # We need to reshape L0 from a square matrix into a row vector to pass into ode45()
        l0 = np.reshape(L0, (81))
        # L must be integrated backwards, so we integrate L(tf - t) from 0 to tf
        initial_condition = [0, tf]
        sol = solve_ivp(self.dldt_minus, [0, tf], l0, dense_output=True)
        t = sol.t
        l = sol.y

        # Reverse time to get L(t) back in forwards time
        t = tf - t
        t = np.flip(t)
        l = np.flip(l, axis=1) # flip in time
        self.l_spline = interp1d(t, l)

    def Ldot(self, t, L):

        x = self.x_d.value(t)
        u = self.u_d.value(t)
        Q = self.Q
        R = self.R

        r = x[0].item()
        alpha = x[1].item()
        beta = x[2].item()
        Vx = x[3].item()
        Vy = x[4].item()
        Vz = x[5].item()
        m = x[6].item()
        phi = x[7].item()
        psi = x[8].item()

        T = u[0].item()
        omega_phi = u[1].item()
        omega_psi = u[2].item()

        A = np.array([  [0,0,0,1,0,0,0,0,0],
                        [-Vy/(r**2*cos(beta)), 				0,	Vy*sin(beta)/(r*cos(beta)**2), 	0, 		1/(r*cos(beta)), 		0, 				0, 							0, 						0],
                        [-Vz/r**2, 							0, 	0, 								0, 		0, 						1/r, 			0, 							0, 						0],
                        [-(Vy**2+Vz**2)/r**2, 				0, 	0, 								0, 		2*Vy/r, 				2*Vz/r, 		-T*sin(phi)/m**2, 			T*cos(phi)/m, 			0],
                        [Vx*Vy/r**2-Vy*Vz*tan(beta)/r**2,	0,	Vy*Vz*(1+tan(beta)**2)/r, 		-Vy/r,	-Vx/r+Vz*tan(beta)/r,	Vy*tan(beta)/r,	-T*cos(phi)*cos(psi)/m**2,	-T*sin(phi)*cos(psi)/m,	-T*cos(phi)*sin(psi)/m],
                        [Vx*Vz/r**2+Vy**2*tan(beta)/r**2, 	0, 	-Vy**2*(1+tan(beta)**2)/r, 		-Vz/r, 	-2*Vy*tan(beta)/r, 		-Vx/r, 			-T*cos(phi)*sin(psi)/m**2,	-T*sin(phi)*sin(psi)/m,	T*cos(phi)*cos(psi)/m],
                        [0, 0, 0, 0, 0, 0, 0, 0,0],
                        [0, 0, 0, 0, 0, 0, 0, 0,0],
                        [0, 0, 0, 0, 0, 0, 0, 0,0]])

        B = np.array([	[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [sin(phi)/m, 0, 0],
                        [cos(phi)*cos(psi)/m, 0, 0],
                        [cos(phi)*sin(psi)/m, 0, 0],
                        [-1/(self.Isp*self.g), 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

        dLdt = np.zeros((9,9))
        # TODO: compute d/dt L(t)
        # print(A)
        # print(B)
        dLdt = (-1/2) * Q.dot(np.transpose(np.linalg.inv(L))) \
                - np.transpose(A).dot(L) + 1/2 * L.dot(np.transpose(L))\
                .dot(B).dot(np.linalg.inv(R)).dot(np.transpose(B)).dot(L)
    
        return dLdt

    def dldt_minus(self, t, l):
        # reshape l to a square matrix
        L = np.reshape(l, (9, 9))

        # compute Ldot
        dLdt_minus = -self.Ldot(t, L)

        # reshape back into a vector
        dldt_minus = np.reshape(dLdt_minus, (81))
        return dldt_minus


    def compute_feedback(self, t, x):

        xditem = self.x_d.value(t)
        xd = np.zeros((9,))
        xbar = np.zeros((9,))
        for i in range(9):
            xd[i] = xditem[i].item()
            xbar[i] = x[i] - xd[i]
        m = xd[6]
        phi = xd[7]
        psi = xd[8]

        B = np.array(([	[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [sin(phi)/m, 0, 0],
                        [cos(phi)*cos(psi)/m, 0, 0],
                        [cos(phi)*sin(psi)/m, 0, 0],
                        [-1/(self.Isp*self.g), 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]]))

        # Retrieve L(t)
        L = np.reshape(self.l_spline(t), (9, 9))

        u_fb = np.zeros((3,))
        # TODO: Compute optimal feedback inputs u_fb using LQR
        u_fb = -np.linalg.inv(self.R).dot(np.transpose(B)).dot(L)\
            .dot(np.transpose(L)).dot(xbar)

        # Add u_fb to u_d(t), the feedforward term. 
        # u = u_fb + u_d
        uditem = self.u_d.value(t)
        ud = np.zeros((3,))
        for i in range(3):
            ud[i] = uditem[i].item()
        u = ud + u_fb;
        return u