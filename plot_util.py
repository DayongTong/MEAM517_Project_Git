import matplotlib.pyplot as plt
import numpy as np

color1 = "dodgerblue"
color2 = "orange"

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
        if idx == 2:
            ax.plot(timesteps1,x_reg[:,idx],color1,label="DIRCON")
            ax.plot(timesteps2,x_dirtrel[:,idx],color2,label="DIRTREL")
            ax.legend(loc="center left")
        else:
            ax.plot(timesteps1,x_reg[:,idx],color1,label="DIRCON")
            ax.plot(timesteps2,x_dirtrel[:,idx],color2,label="DIRTREL")
    else:
        ax.plot(timesteps1, u_reg[:,idx % n_x],color1,label="DIRCON")
        ax.plot(timesteps2,u_dirtrel[:,idx % n_x],color2,label="DIRTREL")
    ax.set(ylabel = labels[idx])
    ax.set(xlabel = "Time (s)")
    plt.tight_layout()
    plt.savefig(fname='optimal_trajectory_plot',dpi=300)
    

def plot_util_overlay_all(timesteps1, x_dircon1, u_dircon1, timesteps2, x_dircon2, u_dircon2, timesteps3, x_dirtrel1, u_dirtrel1, timesteps4, x_dirtrel2, u_dirtrel2, n_x):
    fig, axes = plt.subplots(nrows=4, ncols=3, figsize=(15,15))
    # fig.subplots_adjust(hspace=15)
    # fig.suptitle('Comparison of DIRCON and DIRTREL')
    labels = ['Altitude $r$ (m)', r'Latitude $\alpha$ (deg)', r'Longitude $\beta$ (deg)',
            'X-Velocity $V_x$ (m/s)', 'Y-velocity $V_y$ (m/s)', 'Z-velocity $V_z$ (m/s)',
            'Mass (kg)', r'Pitch $\phi$ (deg)', r'Yaw $\psi$ (deg)',
            'Thrust (N)', r'Pitch Rate $\omega_{\phi}$ (deg/s)', r'Yaw Rate $\omega_{\psi}$ (deg/s)']
    for idx,ax in enumerate(axes.flatten()):
        if idx < n_x:
            if idx == 0:
                l1 = ax.plot(timesteps1, x_dircon1[:,idx],"dodgerblue",linestyle='--', label="optimal DIRCON")
                l2 = ax.plot(timesteps2, x_dircon2[:,idx],"dodgerblue", label="simulated DIRCON")
                l3 = ax.plot(timesteps3, x_dirtrel1[:,idx],"orange",linestyle='--', label="DIRTREL output")
                l4 = ax.plot(timesteps4, x_dirtrel2[:,idx],"orange", label="simulated DIRTREL")
                ax.legend(loc="lower left")
                # ax.legend(handles = [l1,l2,l3,l4] , labels=["optimal DIRCON", "simulated DIRCON", "DIRTREL output","simulated DIRTREL" ],loc='lower center', 
                #     bbox_to_anchor=(0.5, -0.2),fancybox=False, shadow=False, ncol=3)
            else:
                ax.plot(timesteps1, x_dircon1[:,idx],"dodgerblue",linestyle='--', label="optimal DIRCON")
                ax.plot(timesteps2, x_dircon2[:,idx],"dodgerblue", label="simulated DIRCON")
                ax.plot(timesteps3, x_dirtrel1[:,idx],"orange",linestyle='--', label="DIRTREL output")
                ax.plot(timesteps4, x_dirtrel2[:,idx],"orange", label="simulated DIRTREL")
        else:
            ax.plot(timesteps1, u_dircon1[:,idx % n_x],"dodgerblue",linestyle='--', label="optimal DIRCON")   
            ax.plot(timesteps2, u_dircon2[:,idx % n_x],"dodgerblue", label="simulated DIRCON")
            ax.plot(timesteps3, u_dirtrel1[:,idx % n_x],"orange",linestyle='--', label="DIRTREL output")
            ax.plot(timesteps4, u_dirtrel2[:,idx % n_x],"orange", label="simulated DIRTREL")
        ax.set(ylabel = labels[idx])
        ax.set(xlabel = "Time (s)")
        # ax.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(fname='trajectory_plot',dpi=300)
    


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
