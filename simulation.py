# this file simulates a stabilized maneuver of the spacecraft
import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import importlib

import dynamic_constraints
importlib.reload(dynamic_constraints)
from dynamic_constraints import (
    AddCollocationConstraints,
    EvaluateDynamics
)
import spacecraft
importlib.reload(spacecraft)
from spacecraft import Spacecraft


def simulate_spacecraft(x_0,u_0,t_land,spacecraft):
    #simulates stabilized maneuver of spacecraft with initial state x_0
    t_0 = 0.0
    n_points = 1000

    dt = 1e-1

    x = [x_0]
    u = [u_0]
    t = [t_0]
    # x = [x_0.item()]

    while t[-1] < t_land:
        current_time = t[-1]
        current_x = x[-1]
        current_u = spacecraft.compute_feedback(current_time, current_x)

        # Autonomous ODE for constant inputs to work with solve_ivp
        def f(t, x):
            return EvaluateDynamics(current_x, current_u)

        # Integrate one step
        sol = solve_ivp(f, (0, dt), current_x, first_step=dt)

        # Record time, state, and inputs
        t.append(t[-1] + dt)
        x.append(sol.y[:, -1])
        u.append(current_u)

    x = np.array(x)
    u = np.array(u)
    t = np.array(t)
    return x, u, t

if __name__ == '__main__':
    tf = t_land;
    R = np.eye(3);
    Q = np.diag([10, 10, 10, 1, 1, 1, 1, 1, 1]);
    Qf = Q;

    spacecraft = Spacecraft(Q, R, Qf, tf);

    x0 = 0.5 * np.ones((9,)) + x_d(0.0)
    x, u, t = simulate_spacecraft(x0, tf, spacecraft)
    plt.plot(x[:, 0], x[:, 1])

    n_samples = 1000
    t_samples = np.linspace(0.0, tf, n_samples)
    x_des = np.zeros((n_samples, 9))
    for i in range(t_samples.shape[0]):
        x_des[i] = x_d(t_samples[i])
    #plot trajectory
    plt.plot(x_des[:, 0], x_des[:, 1], label='desired trajectory')
    plt.plot(x[:, 0], x[:, 1], label='actual trajectory')
    plt.legend()
    plt.show()
