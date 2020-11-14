# this file simulates a stabilized maneuver of the spacecraft
import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from quadrotor import Quadrotor
from trajectories import *
import matplotlib.pyplot as plt

def simulate_spacecraft(x_0, t_land):
    #simulates stabilized maneuver of spacecraft with initial state x_0
    t_0 = 0.0

    x = [x_0]

    #plot trajectory
    #plot altitude
    #plot latitude
    #plot longitude
    #plot V_x
    #plot V_y
    #plot V_z