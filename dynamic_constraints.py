import numpy as np

from pydrake.solvers.mathematicalprogram import MathematicalProgram
from pydrake.autodiffutils import AutoDiffXd

# calculate the dynamics
# return q_dot, q_ddot as a 6X1
def EvaluateDynamics(x,u):
	pass

# refer to lecture 10 and hw5 set up collocation constraints
# return h_i
def CollocationConstraintEvaluator(dt,x_i,u_i,x_ip1,u_ip1):
	pass

# this function actually use prog.AddConstraint...
# calls collocationconstraintevaluator function
def AddCollocationConstraints(prog,N,x,u,timesteps):
	pass

