import numpy as np
from pydrake.autodiffutils import AutoDiffXd
from math import sin, cos, tan

# def cos(theta):
#   return AutoDiffXd.cos(theta)
# def sin(theta):
#   return AutoDiffXd.sin(theta)
# def tan(theta):
#   return AutoDiffXd.tan(theta)

def EvaluateDynamics(x, u):
    # """
    # Evaluate dynamics for system xdot = f(x, u).
    # Inputs:
    # x (Array[Continuous Variables]): State Variables Array
    #     x = [r, alpha, beta, Vx, Vy, Vz, m, phi, psi]
    # u (Array[Continuous Variables]): Input (control) Variables Array
    #     u = [T, omega_phi, omega_psi]

    # Returns:
    # xdot (Array[Continuous Variables]): Dyanmics of state variables. 
    # """
    g_m = 3.71 # Mars
    g_e = 9.8
    I_sp = 302.39
    
    r = x[0]
    alpha = x[1]
    beta = x[2]
    Vx = x[3]
    Vy = x[4]
    Vz = x[5]
    m = x[6]
    phi = x[7]
    psi = x[8]

    rdot = Vx
    print(beta)
    alphadot = Vy/(r*cos(beta))
    betadot = Vz/r

    T = u[0]
    omega_phi = u[1]
    omega_psi = u[2]

    mdot = -T/(I_sp*g_m)

    Vxdot = T*sin(phi)/m - g_e + (Vy**2 + Vz**2)/r
    Vydot = T*cos(phi)*cos(psi)/m - (Vx*Vy)/r + (Vy*Vz*tan(beta))/r
    Vzdot = T*cos(phi)*sin(psi)/m - (Vx*Vz)/r - (Vy**2*tan(beta))/r

    xdot  = np.array([rdot, alphadot, betadot,
                        Vxdot, Vydot, Vzdot, 
                        mdot, omega_phi, omega_psi])
    return xdot

def CollocationConstraintEvaluator(dt, x_i, u_i, x_ip1, u_ip1):
    # """
    # Computes cubic spline between two knot points i and i+1. 
    # Inputs:
    # dt (Scalar): Length of t_i+1 - t_i
    # x_i (Array[Continuous Variables]): State at time t_i
    # u_i (Array[Continuous Variables]): Input at time t_i
    # x_ip1 (Array[Continuous Variables]): State at time t_i+1
    # u_ip1 (Array[Continuous Variables]): Input at time t_i+1

    # Returns:
    # h_i (Array[Continuous Variables]): The constraint xdot_c - f(x_c, u_c) at midpoint time (t_i+1 - t_i)/2.
    # """
    h_i = np.zeros(10,)
    f_i = EvaluateDynamics(x_i, u_i)
    f_ip1 = EvaluateDynamics(x_ip1, u_ip1)
    x_c = 0.5*(x_i + x_ip1) - dt/8 * (f_ip1 - f_i)
    u_c = 0.5*(u_i + u_ip1)
    h_i = 3/(2*dt)*(x_ip1-x_i) - 0.25*(f_i + f_ip1) - EvaluateDynamics(x_c, u_c)
    return h_i


def AddCollocationConstraints(prog, N, x, u, t_land):
    # """
    # Adds collocation constraints for mathematical program. 
    # Inputs:
    # prog: PyDrake Mathematica Program
    # N (Scalar): Number of knot points
    # x (Array[Continuous Variables]): State variables
    # u (Array[Continuous Variables]): Input variables
    # timesteps (Array[Scalars]): Discretized time points.

    # Return:
    # None
    # """
    dt = t_land/N
    for i in range(N-1):
        h_i = CollocationConstraintEvaluator(dt, x[i], u[i], x[i+1], u[i+1])
        print(i)
        prog.AddLinearEqualityConstraint(h_i, np.zeros(10))


