def logger_util(t_sol, x_sol, u_sol, tinit, xinit, uinit):
    x_labels = ["r: ", "alpha: ", "beta: ", 
              "Vx: ", "Vy: ", "Vz: ",
              "m: ", "phi: ", "psi: "]
    u_labels = ["T: ", "w_phi: ", "w_psi: "]       
    with open("save_solution_variables.txt", mode='w') as F:
        F.write("LOGGING SOLUTIONS... \n")
        F.write("t_sol: ")
        F.write(t_sol[0].astype(str))
        F.write("\n")
        for idx, label in enumerate(x_labels):
            F.write(label)
            F.write(str(x_sol[:,idx]))
            F.write("\n")
        for idx, label in enumerate(u_labels):
            F.write(label)
            F.write(str(u_sol[:,idx]))
            F.write("\n")

        F.write("LOGGING INITIAL GUESSES...\n")
        F.write("Time Initial Guess: ")
        F.write(str(tinit))
        F.write("\n")
        F.write("State Initial Guess: ")
        F.write(str(xinit))
        F.write("\n")
        F.write("Input Initial Guess: ")
        F.write(str(uinit))
