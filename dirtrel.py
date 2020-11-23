def compute_A(x, u):
    r = x[0].item()
    alpha = x[1].item()
    beta = x[2].item()
    Vx = x[3].item()
    Vy = x[4].item()
    Vz = x[5].item()
    m = x[6].item()
    phi = x[7].item()
    psi = x[8].item()

    T = u[0].item()
    omega_phi = u[1].item()
    omega_psi = u[2].item()

    A = np.array([  [0,0,0,1,0,0,0,0,0],
                    [-Vy/(r**2*cos(beta)), 				0,	Vy*sin(beta)/(r*cos(beta)**2), 	0, 		1/(r*cos(beta)), 		0, 				0, 							0, 						0],
                    [-Vz/r**2, 							0, 	0, 								0, 		0, 						1/r, 			0, 							0, 						0],
                    [-(Vy**2+Vz**2)/r**2, 				0, 	0, 								0, 		2*Vy/r, 				2*Vz/r, 		-T*sin(phi)/m**2, 			T*cos(phi)/m, 			0],
                    [Vx*Vy/r**2-Vy*Vz*tan(beta)/r**2,	0,	Vy*Vz*(1+tan(beta)**2)/r, 		-Vy/r,	-Vx/r+Vz*tan(beta)/r,	Vy*tan(beta)/r,	-T*cos(phi)*cos(psi)/m**2,	-T*sin(phi)*cos(psi)/m,	-T*cos(phi)*sin(psi)/m],
                    [Vx*Vz/r**2+Vy**2*tan(beta)/r**2, 	0, 	-Vy**2*(1+tan(beta)**2)/r, 		-Vz/r, 	-2*Vy*tan(beta)/r, 		-Vx/r, 			-T*cos(phi)*sin(psi)/m**2,	-T*sin(phi)*sin(psi)/m,	T*cos(phi)*cos(psi)/m],
                    [0, 0, 0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0, 0, 0,0],
                    [0, 0, 0, 0, 0, 0, 0, 0,0]])
    return A

def compute_B(x, u):
    r = x[0].item()
    alpha = x[1].item()
    beta = x[2].item()
    Vx = x[3].item()
    Vy = x[4].item()
    Vz = x[5].item()
    m = x[6].item()
    phi = x[7].item()
    psi = x[8].item()

    T = u[0].item()
    omega_phi = u[1].item()
    omega_psi = u[2].item()

    B = np.array([  [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    [sin(phi)/m, 0, 0],
                    [cos(phi)*cos(psi)/m, 0, 0],
                    [cos(phi)*sin(psi)/m, 0, 0],
                    [-1/(self.Isp*self.g), 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
    return B

def compute_G(x, u):
    #TODO: compute G

def ell_w(x, u, D, E_1, Q_l, R_l, Ql_n, Q, R, N):
    ell = 0
    for i in range(N-1):
        A[i] = compute_A(x, u)
        B[i] = compute_B(x, u)
        G[i] = compute_G(x, u)

    K = #LQR(A, B, Q, R)
    H[0] = np.zeros()
    E[0] = E_1
    
    for i in range(N-1):
        mat = (Q_l + K[i].T @ R_l @ K[i]) @ E[i]
        ell += mat.trace()
        E[i+1] = (A[i] - B[i] @ K[i]) @ E[i] @ (A[i] - B[i] @ K[i]).T +  (A[i] - B[i] @ K[i]) @ H[i] @ G[i].T + \
                G[i] @ H[i] @  (A[i] - B[i] @ K[i]).T + G[i] @ D @ G[i].T
        H[i+1] =  (A[i] - B[i] @ K[i]) @ H[i] + G[i] @ D

    mat2 = Ql_n @ E[N]
    ell += mat2.trace()
    return ell