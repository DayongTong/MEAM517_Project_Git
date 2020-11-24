import matplotlib.pyplot as plt

def plot_util(timesteps, x_sol, u_sol,n_x):
  fig, axes = plt.subplots(nrows=4, ncols=3, figsize=(15,15))
  fig.subplots_adjust(hspace=10)
  fig.suptitle('Nominal State and Input Trajectories')
  labels = ['Altitude $r$ (m)', r'Latitude $\alpha$ (deg)', r'Longitude $\beta$ (deg)',
            '$V_x$ (m/s)', '$V_y$ (m/s)', '$V_z$ (m/s)',
            'Mass (kg)', r'Pitch $\phi$ (deg)', r'Yaw $\psi$ (deg)',
            'Thrust (N)', r'Pitch Rate $\omega_{\phi}$ (deg/s)', r'Yaw Rate $\omega_{\psi}$ (deg/s)']

  for idx, ax in enumerate(axes.flatten()):
    if idx < n_x:
      ax.plot(timesteps, x_sol[:,idx])
    else:
      ax.plot(timesteps, u_sol[:,idx % n_x])
    ax.set(ylabel = labels[idx])
    ax.set(xlabel = "Simulation Time (s)")

  plt.tight_layout()

def plot_util_spline(N, t_sol, x_s, u_s,n_x):
  fig, axes = plt.subplots(nrows=4, ncols=3, figsize=(15,15))
  fig.subplots_adjust(hspace=10)
  fig.suptitle('Nominal State and Input Trajectories')
  labels = ['Altitude $r$ (km)', r'Latitude $\alpha$ (deg)', r'Longitude $\beta$ (deg)',
            '$V_x$ (m/s)', '$V_y$ (m/s)', '$V_z$ (m/s)',
            'Mass (kg)', r'Pitch $\phi$ (deg)', r'Yaw $\psi$ (deg)',
            'Thrust (N)', r'Pitch Rate $\omega_{\phi}$ (deg/s)', r'Yaw Rate $\omega_{\psi}$ (deg/s)']
  
  for idx, ax in enumerate(axes.flatten()):
    if idx < n_x:
      vals = []
      for i in range(int(N*t_sol[0])):
        vals.append(x_s.value(i)[idx])
        ax.plot(vals)
    else:
      vals = []
      for i in range(int(N*t_sol[0])):
        vals.append(u_s.value(i)[idx % n_x])
        ax.plot(vals)
    ax.set(ylabel = labels[idx])
    ax.set(xlabel = "Simulation Time (s)")

  plt.tight_layout()

def plot_3d_trajectory(r_sol, alpha_sol, beta_sol, T_sol=None):
  import matplotlib as mpl
  from mpl_toolkits.mplot3d import Axes3D
  import numpy as np
  import matplotlib.pyplot as plt
  mpl.rcParams['legend.fontsize'] = 10
  fig = plt.figure()
  ax = fig.gca(projection='3d')
  x = r_sol*np.sin(beta_sol)*np.cos(alpha_sol)
  y = r_sol*np.sin(beta_sol)*np.sin(alpha_sol)
  z = r_sol*np.cos(beta_sol)
  ax.plot(x, y, z, label='Rocket Trajectory')
  ax.set_xlabel('x position')
  ax.set_ylabel('y position')
  ax.set_zlabel('z [km]')  
  ax.set_zlim(0,4)
  ax.legend()
  plt.tight_layout()
  plt.show()