import numpy as np
from pydrake.autodiffutils import AutoDiffXd

from pydrake.solvers.mathematicalprogram import MathematicalProgram

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)

# set up initial conditions based on the paper
def AddInitialConstraints(prog,x,u):

    for i in range(len(initial_state)):
        prog.AddLinearEqualityConstraint(x[i], initial_state[i])

    prog.AddLinearEqualityConstraint(u[0], T_0)
    prog.AddLinearEqualityConstraint(u[1], phiddot_0)
    prog.AddLinearEqualityConstraint(u[2], psiddot_0)


# add landing constraints so that it lands in desire position
# you might need to make this a function handler thing and have another function call this like in hw5
def AddLandingConstraints(prog,x,u):
    prog.AddLinearEqualityConstraint(x[0], r_N)
    prog.AddLinearEqualityConstraint(x[1], alpha_N)
    prog.AddLinearEqualityConstraint(x[2], beta_N)
    prog.AddLinearEqualityConstraint(x[3], Vx_N)
    prog.AddLinearEqualityConstraint(x[4], Vy_N)
    prog.AddLinearEqualityConstraint(x[5], Vz_N)
    prog.AddConstraint(x[6] >= 1000) #TODO: change to whatever we set our dry weight to be
    prog.AddLinearEqualityConstraint(x[7], phi_N)
    prog.AddLinearEqualityConstraint(x[8], psi_N)
    prog.AddLinearEqualityConstraint(u[0], T_N)
    prog.AddLinearEqualityConstraint(u[1], phiddot_N)
    prog.AddLinearEqualityConstraint(u[2], psiddot_N)




