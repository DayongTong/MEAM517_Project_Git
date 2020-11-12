import numpy as np
from pydrake.autodiffutils import AutoDiffXd

from pydrake.solvers.mathematicalprogram import MathematicalProgram

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)

# set up initial conditions based on the paper
def AddInitialConstraints(prog,x,u):
    r_curr = x[0]
    alpha_curr = x[1]
    beta_curr = x[2]
    phi_curr = x[3]
    psi_curr = x[4]
    rdot_curr = x[5]
    alphadot_curr = x[6]
    betadot_curr = x[7]
    phidot_curr = x[8]
    psidot_curr = x[9]
    thrust_curr = u[0]
    phiddot_curr = u[1]
    psiddot_curr = u[2]

    alpha_curr = alpha_0
    beta_curr = beta_0
    phi_curr = phi_0
    psi_curr = psi_0
    r_curr = r_0
    rdot_curr = 0
    thrust_curr = T_0
    phiddot_curr = phiddot_0
    psiddot_curr = psiddot_0

    prog.AddLinearEqualityConstraint(alphadot_curr*r_curr*cos(beta_curr), 0)
    prog.AddLinearEqualityConstraint(betadot_curr*r_curr, 0)

# add landing constraints so that it lands in desire position
# you might need to make this a function handler thing and have another function call this like in hw5
def AddLandingConstraints(prog,x,u):
    r_fin = x[0]
    alpha_fin = x[1]
    beta_fin = x[2]
    phi_fin = x[3]
    psi_fin = x[4]
    rdot_fin = x[5]
    alphadot_fin = x[6]
    betadot_fin = x[7]
    thrust_fin = u[0]
    phiddot_fin = u[1]
    psiddot_fin = u[2]

    alpha_fin = alpha_N
    beta_fin = beta_N
    phi_fin = phi_N
    psi_fin = psi_N
    r_fin = r_N
    rdot_fin = 0
    thrust_fin = T_N
    phiddot_fin = phiddot_N
    psiddot_fin = psiddot_N


    prog.AddLinearEqualityConstraint(alphadot_fin*r_fin*cos(beta_fin), 0)
    prog.AddLinearEqualityConstraint(betadot_fin*r_fin, 0)




