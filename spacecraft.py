# this file construct a spacecraft object that can compute the optimal feedback controller

# TESTING SYNC ATTENSION PLZZZZ


import numpy as np
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp

class Spacecraft(object):
  '''
  Constructor. Compute function S(t) using S(t) = L(t) L(t)^t, by integrating backwards
  from S(tf) = Qf. We will then use S(t) to compute the optimal controller efforts in 
  the compute_feedback() function
  '''
  def __init__(self, Q, R, Qf, tf,x_d,u_d):
    self.m = 1
    self.a = 0.25
    self.I = 0.0625
    self.Q = Q
    self.R = R

    ''' 
    We are integrating backwards from Qf
    '''

    # Get L(tf) L(tf).T = S(tf) by decomposing S(tf) using Cholesky decomposition
    L0 = cholesky(Qf).transpose()

    # We need to reshape L0 from a square matrix into a row vector to pass into ode45()
    l0 = np.reshape(L0, (36))
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

    x = x_d(t)
    u = u_d(t)
    Q = self.Q
    R = self.R
    

    A = np.array(([[0,0,0,1,0,0],
                   [0,0,0,0,1,0],
                   [0,0,0,0,0,1],
                   [0,0,-np.cos(x[2])/self.m * (u[0]+u[1]),0,0,0],
                   [0,0,-np.sin(x[2])/self.m * (u[0]+u[1]),0,0,0],
                   [0,0,0,0,0,0]]))
    B = np.array(([0,0],
                  [0,0],
                  [0,0],
                  [-np.sin(x[2])/self.m, -np.sin(x[2])/self.m],
                  [np.cos(x[2])/self.m, np.cos(x[2])/self.m],
                  [self.a/self.I,-self.a/self.I]))

    dLdt = np.zeros((6,6))
    # TODO: compute d/dt L(t)
    dLdt = (-1/2) * Q.dot(np.transpose(np.linalg.inv(L))) \
           - np.transpose(A).dot(L) + 1/2 * L.dot(np.transpose(L))\
           .dot(B).dot(np.linalg.inv(R)).dot(np.transpose(B)).dot(L)
    
    return dLdt

  def dldt_minus(self, t, l):
    # reshape l to a square matrix
    L = np.reshape(l, (6, 6))

    # compute Ldot
    dLdt_minus = -self.Ldot(t, L)

    # reshape back into a vector
    dldt_minus = np.reshape(dLdt_minus, (36))
    return dldt_minus


  def compute_feedback(self, t, x):
    xbar = x - x_d(t)
    xd = x_d(t)

    B = np.array(([0,0],
                  [0,0],
                  [0,0],
                  [-np.sin(xd[2])/self.m, -np.sin(xd[2])/self.m],
                  [np.cos(xd[2])/self.m, np.cos(xd[2])/self.m],
                  [self.a/self.I,-self.a/self.I]))

    # Retrieve L(t)
    L = np.reshape(self.l_spline(t), (6, 6))

    u_fb = np.zeros((2,))
    # TODO: Compute optimal feedback inputs u_fb using LQR
    u_fb = -np.linalg.inv(self.R).dot(np.transpose(B)).dot(L)\
           .dot(np.transpose(L)).dot(xbar)

    # Add u_fb to u_d(t), the feedforward term. 
    # u = u_fb + u_d
    u = u_d(t) + u_fb;
    return u