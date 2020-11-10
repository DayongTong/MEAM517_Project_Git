import numpy as np
from pydrake.autodiffutils import AutoDiffXd

from pydrake.solvers.mathematicalprogram import MathematicalProgram

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)

# set up initial conditions based on the paper
def AddInitialConstraints(prog,x,u):
	pass

# add landing constraints so that it lands in desire position
# you might need to make this a function handler thing and have another function call this like in hw5
def AddLandingConstraints(prog,x,u):
	pass



