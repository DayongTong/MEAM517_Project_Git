def compute_A(x, u, w, i):
    r = x[:,0]
    alpha = x[:,1]
    beta = x[:,2]
    Vx = x[:,3]
    Vy = x[:,4]
    Vz = x[:,5]
    m = x[:,6]
    phi = x[:,7]
    psi = x[:,8]

    T = u[:,0]
    omega_phi = u[:,1]
    omega_psi = u[:,2]
    A = np.array([  [0,0,0,1,0,0,0,0,0],
                    [-Vy[i]/(r[i]**2*drake_math.cos(beta[i])), 				0,	Vy[i]*drake_math.sin(beta[i])/(r[i]*drake_math.cos(beta[i])**2), 	0, 		1/(r[i]*drake_math.cos(beta[i])), 		0, 				0, 							0, 						0],
                    [-Vz[i]/r[i]**2, 							0, 	0, 								0, 		0, 						1/r[i], 			0, 							0, 						0],
                    [-(Vy[i]**2+Vz[i]**2)/r[i]**2, 				0, 	0, 								0, 		2*Vy[i]/r[i], 				2*Vz[i]/r[i], 		-T[i]*drake_math.sin(phi[i])/(m[i]+w)**2, 			T[i]*drake_math.cos(phi[i])/(m[i]+w), 			0],
                    [Vx[i]*Vy[i]/r[i]**2-Vy[i]*Vz[i]*drake_math.tan(beta[i])/r[i]**2,	0,	Vy[i]*Vz[i]*(1+drake_math.tan(beta[i])**2)/r[i], 		-Vy[i]/r[i],	-Vx[i]/r[i]+Vz[i]*drake_math.tan(beta[i])/r[i],	Vy[i]*drake_math.tan(beta[i])/r[i],	-T[i]*drake_math.cos(phi[i])*drake_math.cos(psi[i])/(m[i]+w)**2,	-T[i]*drake_math.sin(phi[i])*drake_math.cos(psi[i])/(m[i]+w),	-T[i]*drake_math.cos(phi[i])*drake_math.sin(psi[i])/(m[i]+w)],
                    [Vx[i]*Vz[i]/r[i]**2+Vy[i]**2*drake_math.tan(beta[i])/r[i]**2, 	0, 	-Vy[i]**2*(1+drake_math.tan(beta[i])**2)/r[i], 		-Vz[i]/r[i], 	-2*Vy[i]*drake_math.tan(beta[i])/r[i], 		-Vx[i]/r[i], 			-T[i]*drake_math.cos(phi[i])*drake_math.sin(psi[i])/(m[i]+w)**2,	-T[i]*drake_math.sin(phi[i])*drake_math.sin(psi[i])/(m[i]+w),	T[i]*drake_math.cos(phi[i])*drake_math.cos(psi[i])/(m[i]+w)],
                    [0, 0, 0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0, 0, 0,0]])

    return A

def compute_B(x, u, w, i):

    m = x[:,6]
    phi = x[:,7]
    psi = x[:,8]

    B = np.array([  [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    [drake_math.sin(phi[i])/(m[i]+w), 0, 0],
                    [drake_math.cos(phi[i])*drake_math.cos(psi[i])/(m[i]+w), 0, 0],
                    [drake_math.cos(phi[i])*drake_math.sin(psi[i])/(m[i]+w), 0, 0],
                    [-1/(Isp*g), 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
    return B

def compute_G(x, u, w, i):
    m = x[:,6]
    phi = x[:,7]
    psi = x[:,8]
    T = u[:,0]
    G = np.array([ [0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,-T[i]*drake_math.sin(phi[i])/(m[i]+w)**2,0,0],
                   [0,0,0,0,0,0,-T[i]*drake_math.cos(phi[i])*drake_math.cos(psi[i])/(m[i]+w)**2,0,0],
                   [0,0,0,0,0,0,-T[i]*drake_math.cos(phi[i])*drake_math.sin(psi[i])/(m[i]+w)**2,0,0],
                   [0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0]])
    return G

def ell_w(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N, start=0, stop=6):
    
    ell = 0
    w = 20*np.random.rand() # [0, 1)
    A = np.zeros((N,n_x,n_x), dtype=AutoDiffXd)
    B = np.zeros((N,n_x,n_u), dtype=AutoDiffXd)
    G = np.zeros((N,n_x,n_x), dtype=AutoDiffXd)

    K = [0]*(N-1)

    K = np.zeros((N-1,n_u,n_x), dtype=AutoDiffXd)    
    E = np.zeros((N,n_x,n_x), dtype=AutoDiffXd)    
    H = np.zeros((N,n_x,n_x), dtype=AutoDiffXd)    
    for i in range(N):
        A[i] = compute_A(x, u, w, i)
        B[i] = compute_B(x, u, w, i)
        G[i] = compute_G(x, u, 0, i)
    P = [0]*N
    P[-1] = Ql_n
    
    # compute Riccati difference equation for i in 1:N
    for i in reversed(range(1,N)):
        P[i-1] = Q + A[i].T@P[i]@A[i]-A[i].T@P[i]@B[i]@(R + B[i].T@P[i]@B[i]).T@(B[i].T@P[i]@A[i])
    for i in range(N-1):
        K[i] = (R + B[i].T@P[i+1]@B[i]).T@(B[i].T@P[i+1]@A[i])
    
    H[0] = np.zeros_like(Q)
    E[0] = E_1
    for i in range(start, stop):
        ell += np.trace(Q_l + K[i].T @ R_l @ K[i] @ E[i])
        E[i+1] = (A[i] - B[i] @ K[i]) @ E[i] @ (A[i] - B[i] @ K[i]).T + \
                 (A[i] - B[i] @ K[i]) @ H[i] @ G[i].T + \
                  G[i] @ H[i].T @  (A[i] - B[i] @ K[i]).T + G[i] @ D @ G[i].T
        H[i+1] =  (A[i] - B[i] @ K[i]) @ H[i] + G[i] @ D
    ell += np.trace(Ql_n @ E[-1])
    return ell

def AddCost(x, u, use_dirtrel=False):
    TT = 0
    for i in range(N-1):
        TT += (u[i,0]**2 + u[i+1,0]**2)
    if use_dirtrel == False:
        prog.AddQuadraticCost(TT)
    else:
        D = 20*np.eye(9)
        E_1 = np.eye(9)
        TT += ell_w(x, u, D, E_1, Q, R,Q,Q,R,N,0,6)
        TT += ell_w(x, u, D, E_1, Q, R,Q,Q,R,N,6,N-1)
        prog.AddCost(TT)
