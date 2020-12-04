

def ComputeRicattiEq(x, u, D, E_l, Q_l, R_l, Ql_n, Q, R, N):
    w = 20
    A = np.zeros((N,n_x,n_x), dtype=AutoDiffXd)
    B = np.zeros((N,n_x,n_u), dtype=AutoDiffXd)
    G = np.zeros((N,n_x,n_x), dtype=AutoDiffXd)
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
        P[i-1] = Q + A[i].T@P[i]@A[i]-A[i].T@P[i]@B[i]@inv(R + B[i].T@P[i]@B[i])@(B[i].T@P[i]@A[i])
    for i in range(N-1):
        K[i] = inv(R + B[i].T@P[i+1]@B[i])@(B[i].T@P[i+1]@A[i])
    
    H[0] = np.zeros_like(Q)
    E[0] = E_1
    x_wp = np.zeros((N,n_x), dtype=AutoDiffXd)
    x_wn = np.zeros((N,n_x), dtype=AutoDiffXd)
    u_wp = np.zeros((N,n_u), dtype=AutoDiffXd)
    u_wn = np.zeros((N,n_u), dtype=AutoDiffXd)

    for i in range(N-1):
        E[i+1] = (A[i] - B[i] @ K[i]) @ E[i] @ (A[i] - B[i] @ K[i]).T + \
                 (A[i] - B[i] @ K[i]) @ H[i] @ G[i].T + \
                  G[i] @ H[i].T @  (A[i] - B[i] @ K[i]).T + G[i] @ D @ G[i].T
        H[i+1] =  (A[i] - B[i] @ K[i]) @ H[i] + G[i] @ D

    return P, K, E, H

def AddEllipsoidalConstraints(prog,x,u,D,E_1,Q_l,R_l,Ql_n,Q,R,N):

    def AddEllipsoidalConstraintsHelper_xwp(vars):
        xvar = np.zeros((N,n_x),dtype=AutoDiffXd)
        uvar = np.zeros((N,n_u),dtype=AutoDiffXd)
        for i in range(n_x+n_u):
            if i < n_x:
                xvar[:,i] = vars[i*N:(i+1)*N]
            else:
                uvar[:,i-n_x] = vars[i*N:(i+1)*N]
        return EllipsoidConstraintEvaluator_xwp(xvar,uvar,D,E_1,Q_l,R_l,Ql_n,Q,R,N)

    flat = np.hstack((np.hstack((x,u))[:,0:9].flatten('F'),np.hstack((x,u))[:,9:12].flatten('F')))
    flat = flat.reshape((N*(n_x+n_u),1))
    lox = ncycles([r_min,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,0,phi_min,psi_min], N)
    hix = ncycles([r_max,np.inf,np.inf,np.inf,np.inf,np.inf,m_0,phi_max,psi_max], N)
    lou = ncycles([phidot_min, psidot_min, T_min],N)
    hiu = ncycles([phidot_max, psidot_max, T_max],N)
    lox.extend(lou)
    hix.extend(hiu)
    prog.AddConstraint(AddEllipsoidalConstraintsHelper_xwp,
                       lox, 
                       hix,
                       vars=flat)

def EllipsoidConstraintEvaluator_xwp(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N):
    x_wp = np.zeros_like(x)
    P, K, E, H = ComputeRicattiEq(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N)
    print(x[1,:].shape)
    print(E[1].shape)
    print(x_wp.shape)
    for i in range(N):
        for j in range(9):
            print(i)
            x_wp[i:,] = x[i:,] + E[i,j]**0.5

    return x_wp

def EllipsoidConstraintEvaluator_xwn(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N):
    x_wn = np.zeros((N,n_x),dtype='object')
    P, K, E, H = ComputeRicattiEq(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N)
    for i in range(9):
        x_wn[i:,] = x[i:,] - E[:,i]**0.5

    return x_wn

def EllipsoidConstraintEvaluator_uwp(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N):
    u_wp = np.zeros((N,n_u),dtype='object')
    P, K, E, H = ComputeRicattiEq(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N)
    for i in range(9):
        u_wp[i:,] = u[i:,] + ((K[i]@E[i]@K[i].T)[:,i])**0.5

    return u_wp

def EllipsoidConstraintEvaluator_uwn(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N):
    u_wn = np.zeros((N,n_u),dtype='object')
    P, K, E, H = ComputeRicattiEq(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N)
    for i in range(9):
        u_wn[i:,] = u[i:,] - ((K[i]@E[i]@K[i].T)[:,i])**0.5
    return u_wn
