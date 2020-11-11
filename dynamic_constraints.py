import numpy as np
from math import sin, cos, tan
from pydrake.solvers.mathematicalprogram import MathematicalProgram
from pydrake.autodiffutils import AutoDiffXd

# calculate the dynamics
# return q_dot, q_ddot as a 10X1
def EvaluateDynamics(x,u):
	m = 1
	g = 3.71 # Mars
	x[0] = r
	x[1] = alpha
	x[2] = beta
	x[3] = phi
	x[4] = psi

	x[5] = rdot
	x[6] = alphadot
	x[7] = betadot
	x[8] = phidot
	x[9] = psidot

	u[0] = T
	u[1] = u_phi
	u[2] = u_psi

	r_ddot      = T*sin(phi)/m - g + r*alphadot**2*cos(beta)**2 + r*betadot**2
	alpha_ddot  = T*cos(phi)*cos(psi)/(m * r * cos(beta)) - 2*alphadot*rdot/r + 2*alphadot*beta*tan(beta)
	beta_ddot   = T*cos(phi)*sin(psi)/(m*r) - 2*betadot*rdot/r - 0.5*alphadot**2*sin(2*beta) 
	xdot = np.array([rdot, alphadot, betadot, phidot, psidot,
					r_ddot, alpha_ddot, beta_ddot, u_phi, u_psi])
	return xdot

# refer to lecture 10 and hw5 set up collocation constraints
# return h_i
def CollocationConstraintEvaluator(dt,x_i,u_i,x_ip1,u_ip1):
	h_i = np.zeros(10,)

	f_i = EvaluateDynamics(x_i, u_i)
	f_ip1 = EvaluateDynamics(x_ip1, u_ip1)
	x_c = 0.5*(x_i + x_ip1) - dt/8 * (f_ip1 - f_i)
	u_c = 0.5*(u_i + u_ip1)
	h_i = 3/(2*dt)*(x_ip1-x_i) - 0.25*(f_i + f_ip1) - EvaluateDynamics(x_c, u_c)
	return h_i

# this function actually use prog.AddConstraint...
# calls collocationconstraintevaluator function
def AddCollocationConstraints(prog,N,x,u,timesteps):
	for i in range(N-1):
		dt = timesteps[i+1]-timesteps[i]
		h_i = CollocationConstraintEvaluator(dt, x[i], u[i], x[i+1], u[i+1])
		prog.AddLinearEqualityConstraint(h_i, np.zeros(10))




