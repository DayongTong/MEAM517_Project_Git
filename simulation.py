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

    dt = 3e-4

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

