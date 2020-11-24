import numpy as np
from pydrake.autodiffutils import AutoDiffXd
import pydrake.math as drake_math
from math import sin, cos, tan

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
    alphadot = Vy/(r*drake_math.cos(beta))
    betadot = Vz/r

    T = u[0]
    omega_phi = u[1]
    omega_psi = u[2]

    mdot = -T/(I_sp*g_m)

    Vxdot = T*drake_math.sin(phi)/m - g_m + (Vy**2 + Vz**2)/r
    Vydot = T*drake_math.cos(phi)*drake_math.cos(psi)/m - (Vx*Vy)/r + (Vy*Vz*drake_math.tan(beta))/r
    Vzdot = T*drake_math.cos(phi)*drake_math.sin(psi)/m - (Vx*Vz)/r - (Vy**2*drake_math.tan(beta))/r

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
    # h_i (Array[Continuous Variables]): The constraint xdot_c - f(x_c, u_c) 
    # at midpoint time (t_i+1 - t_i)/2.
    # """

    f_i = EvaluateDynamics(x_i, u_i)
    f_ip1 = EvaluateDynamics(x_ip1, u_ip1)
    x_c = 0.5*(x_i + x_ip1) - dt/8 * (f_ip1 - f_i)
    u_c = 0.5*(u_i + u_ip1)
    h_i = 3/(2*dt)*(x_ip1-x_i) - 0.25*(f_i + f_ip1) - EvaluateDynamics(x_c, u_c)
    return h_i


def AddCollocationConstraints(prog, N, x, u, dt):
    # """
    # Adds collocation constraints for mathematical program. 
    # Inputs:
    # prog: PyDrake Mathematical Program
    # N (Scalar): Number of knot points
    # x (Array[Continuous Variables]): State variables
    # u (Array[Continuous Variables]): Input variables
    # timesteps (Array[Scalars]): Discretized time points.

    # Return:
    # None
    # """
    n_x = 9
    n_u = 3
    for i in range(N - 1):
      def CollocationConstraintHelper(vars):
        dt = vars[-1]
        x_i = vars[:n_x]
        u_i = vars[n_x:n_x + n_u]
        x_ip1 = vars[n_x + n_u: 2*n_x + n_u]
        u_ip1 = vars[-n_u-1:-1]

        return CollocationConstraintEvaluator(dt, x_i, u_i, x_ip1, u_ip1)

      prog.AddConstraint(CollocationConstraintHelper, np.zeros_like(x[i,:]), np.zeros_like(x[i,:]), vars=np.hstack((x[i,:], u[i,:], x[i+1,:], u[i+1,:], dt)))

def numeric_eval_dynamics(x_in, u_in,time,N):
    g_m = 3.71 # Mars
    I_sp = 302.39
    xdot = []
    for i in range(1,int(N*time)):
        x = x_in.value(i)
        u = u_in.value(i)

        r = x[0].item()
        alpha = x[1].item()
        beta = x[2].item()
        Vx = x[3].item()
        Vy = x[4].item()
        Vz = x[5].item()
        m = x[6].item()
        phi = x[7].item()
        psi = x[8].item()

        rdot = Vx
        alphadot = Vy/(r*np.cos(beta))
        betadot = Vz/r

        T = u[0].item()
        omega_phi = u[1].item()
        omega_psi = u[2].item()

        mdot = -T/(I_sp*g_m)

        Vxdot = T*np.sin(phi)/m - g_m + (Vy**2 + Vz**2)/r
        Vydot = T*np.cos(phi)*np.cos(psi)/m - (Vx*Vy)/r + (Vy*Vz*np.tan(beta))/r
        Vzdot = T*np.cos(phi)*np.sin(psi)/m - (Vx*Vz)/r - (Vy**2*np.tan(beta))/r

        temp  = [rdot, alphadot, betadot,
                            Vxdot, Vydot, Vzdot, 
                            mdot, omega_phi, omega_psi]
        xdot.append(temp)
    return np.array(xdot)
