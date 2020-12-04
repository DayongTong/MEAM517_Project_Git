import matplotlib.pyplot as plt
import numpy as np

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

def plot_util_overlay(timesteps1,x_reg,u_reg,timesteps2,x_dirtrel,u_dirtrel,n_x):
  fig,axes = plt.subplots(nrows=4,ncols=3,figsize=(15,15))
  # fig.subplots_adjust(hspace=10)
  fig.suptitle('Trajectories of simulation')
  labels = ['Altitude $r$ (m)', r'Latitude $\alpha$ (deg)', r'Longitude $\beta$ (deg)',
            '$V_x$ (m/s)', '$V_y$ (m/s)', '$V_z$ (m/s)',
            'Mass (kg)', r'Pitch $\phi$ (deg)', r'Yaw $\psi$ (deg)',
            'Thrust (N)', r'Pitch Rate $\omega_{\phi}$ (deg/s)', r'Yaw Rate $\omega_{\psi}$ (deg/s)']
    
  for idx,ax in enumerate(axes.flatten()):
    if idx < n_x:
      ax.plot(timesteps1,x_reg[:,idx],label="DIRCON")
      ax.plot(timesteps2,x_dirtrel[:,idx],label="DIRTREL")
    else:
      ax.plot(timesteps1, u_reg[:,idx % n_x],label="DIRCON")
      ax.plot(timesteps2,u_dirtrel[:,idx % n_x],label="DIRTREL")
    ax.set(ylabel = labels[idx])
    ax.set(xlabel = "Time (s)")
    ax.legend(loc="lower right")


def plot_util_spline(N, t_sol, x_s, u_s, n_x, r_min):
  fig, axes = plt.subplots(nrows=4, ncols=3, figsize=(15,15))
  fig.subplots_adjust(hspace=10)
  fig.suptitle('Nominal State and Input Trajectories')
  labels = ['Altitude $r$ (m)', r'Latitude $\alpha$ (deg)', r'Longitude $\beta$ (deg)',
            '$V_x$ (m/s)', '$V_y$ (m/s)', '$V_z$ (m/s)',
            'Mass (kg)', r'Pitch $\phi$ (deg)', r'Yaw $\psi$ (deg)',
            'Thrust (N)', r'Pitch Rate $\omega_{\phi}$ (deg/s)', r'Yaw Rate $\omega_{\psi}$ (deg/s)']
  timevec = np.linspace(0,N*t_sol[0],500)
  for idx, ax in enumerate(axes.flatten()):
    vals = []
    if idx < n_x:
      for i in timevec:
        val = x_s.value(i)[idx]
        if idx == 0:
          val -= r_min
          val /= 1000
        elif idx == 1 or idx == 2 or idx == 7 or idx == 8:
          val *= 180/np.pi
        vals.append(val)
      ax.plot(timevec,vals,'g')
    else:
      for i in timevec:
        val = u_s.value(i)[idx % n_x]
        if idx == 10 or idx == 11:
          val *= 180/np.pi
        vals.append(val)
      ax.plot(timevec,vals,'g')
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