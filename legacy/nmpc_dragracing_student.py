from sim import *
from utils import *

def nmpc_controller(kappa_table = None):
    T = 30 
    h = 0.1
    N = int(np.ceil(T / h))

    ###################### Modeling Start ######################
    # system dimensions
    Dim_state = 6
    Dim_ctrl  = 2
    Dim_aux   = 4 

    xm = ca.MX.sym('xm', (Dim_state, 1))
    um = ca.MX.sym('um', (Dim_ctrl, 1))
    zm = ca.MX.sym('zm', (Dim_aux, 1)) # F_yf F_yr | F_muf F_mur

    ## rename the control inputs
    Fx = um[0]
    delta = um[1]

    ## air drag
    Fd = param["Frr"] + param["Cd"] * xm[0]**2
    Fd = Fd * ca.tanh(- xm[0] * 100)
    Fb = 0.0
    ## 
    
    af, ar   = get_slip_angle(xm[0], xm[1], xm[2], delta, param)
    Fzf, Fzr = normal_load(Fx, param)
    Fxf, Fxr = chi_fr(Fx)

    Fyf = tire_model_sim(af, Fzf, Fxf, param["C_alpha_f"], param["mu_f"])
    Fyr = tire_model_sim(ar, Fzr, Fxr, param["C_alpha_r"], param["mu_r"])

    dUx  = (Fxf * ca.cos(delta) - zm[0] * ca.sin(delta) + Fxr + Fd) / param["m"] + xm[2] * xm[1]
    dUy  = (Fyf * ca.cos(delta) + zm[1] * ca.sin(delta) + Fyr + Fb) / param["m"] - xm[2] * xm[0]
    dr   = (param["L_f"] * (Fyf * ca.cos(delta) + Fxf * ca.sin(delta)) - param["L_r"] * Fyr) / param["Izz"] 
    
    dx   = ca.cos(xm[5]) * xm[0] - ca.sin(xm[5]) * xm[1]
    dy   = ca.sin(xm[5]) * xm[0] + ca.cos(xm[5]) * xm[1]
    dyaw = xm[2]
    xdot = ca.vertcat(dUx, dUy, dr, dx, dy, dyaw)

    xkp1 = xdot * h + xm
    Fun_dynmaics_dt = ca.Function('f_dt', [xm, um, zm], [xkp1])

    # enforce constraints for auxiliary variable z[0] = Fyf
    alg  = ca.vertcat(Fyf,Fyr)
    Fun_alg = ca.Function('alg', [xm, um, zm], [alg])
    
    ###################### MPC variables ######################
    x = ca.MX.sym('x', (Dim_state, N + 1))
    u = ca.MX.sym('u', (Dim_ctrl, N))
    z = ca.MX.sym('z', (Dim_aux, N))
    p = ca.MX.sym('p', (Dim_state, 1))

    ###################### MPC constraints start ######################
    ## MPC equality constraints ##
    cons_dynamics = []
    for k in range(N):
        xkp1 = Fun_dynmaics_dt(x[:, k], u[:, k], z[:, k])
        Fy2  = Fun_alg(x[:, k], u[:, k], z[:, k])
        for j in range(Dim_state):
            cons_dynamics.append(x[j, k+1] - xkp1[j])
        for j in range(2):
            cons_dynamics.append(Fy2[j])
    
    ## MPC inequality constraints ##
    # G(x) <= 0
    cons_ineq = []

    ## state / inputs limits:
    for k in range(N):
        cons_ineq.append(-x[0,k]+2)
        cons_ineq.append(u[0,k]-param["Peng"]/x[0,k])
        cons_ineq.append(1-x[3,k]*x[3,k]/100-x[4,k]*x[4,k]/100)

    ## friction cone constraints
    for k in range(N):
        Fx    = u[0, k]; delta = u[1, k]
        af, ar = get_slip_angle(xm[0], xm[1], xm[2], delta, param) 
        Fzf, Fzr = normal_load(Fx, param)
        Fxf, Fxr = chi_fr(Fx) 

        Fyf = tire_model_sim(af, Fzf, Fxf, param["C_alpha_f"], param["mu_f"])
        Fyr = tire_model_sim(ar, Fzr, Fxr, param["C_alpha_r"], param["mu_r"])

        cons_ineq.append(Fxf**2 + Fyf**2 - (param["mu_f"]*Fzf)**2 - z[2,k]**2)
        cons_ineq.append(Fxr**2 + Fyr**2 - (param["mu_r"]*Fzr)**2 - z[3,k]**2)        

    ###################### MPC cost start ######################
    ## cost function design
    J = 0.0
    J = J + x[4,N]**2 + x[5,k]**2
    
    ## road tracking 
    for k in range(N):
        J = J + x[4,k]**2 + x[5,k]**2 
 
    ## excessive slip angle / friction
    for k in range(N):
        Fx = u[0, k]; delta = u[1, k]
        af, ar   = get_slip_angle(xm[0], xm[1], xm[2], delta, param)
        #af, ar = get_slip_angle(# TODO)
        Fzf, Fzr = normal_load(Fx, param)
        Fxf, Fxr = chi_fr(Fx)

        xi = 0.85
        F_offset = 2000 ## a slacked ReLU function using only sqrt()
        Fyf_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2
        Fyf_max_sq = (ca.sqrt( Fyf_max_sq**2 + F_offset) + Fyf_max_sq) / 2
        Fyf_max = ca.sqrt(Fyf_max_sq)

        ## modified front slide sliping angle
        alpha_mod_f = ca.arctan(3 * Fyf_max / param["C_alpha_f"] * xi)

        Fyr_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2
        Fyr_max_sq = (ca.sqrt( Fyr_max_sq**2 + F_offset) + Fyr_max_sq) / 2
        Fyr_max = ca.sqrt(Fyr_max_sq)

        ## modified rear slide sliping angle
        alpha_mod_r = ca.arctan(3 * Fyr_max / param["C_alpha_r"] * xi)

        ## limit friction penalty
        J = J #+ ## TODO: Avoid front tire saturation
        J = J #+ ## TODO: Avoid  rear tire saturation
        J = J #+ ## TODO: Penalize slack variable for friction cone limits

    # initial condition as parameters
    cons_init = [x[:, 0] - p]
    ub_init_cons = np.zeros((Dim_state, 1))
    lb_init_cons = np.zeros((Dim_state, 1))

    state_ub = np.array([ 1e2,  1e2,  1e2,  1e8,  1e8,  1e8])
    state_lb = np.array([-1e2, -1e2, -1e2, -1e8, -1e8, -1e8])
    ctrl_ub  = np.array([ 1e8, param["delta_max"]]) ## traction force | steering angle, use param["delta_max"]
    ctrl_lb  = np.array([ param["Peng"]/2, - param["delta_max"]])
    aux_ub   = np.array([ 1e5,  1e5,  1e5,  1e5])
    aux_lb   = np.array([-1e5, -1e5, -1e5, -1e5])

    lb_dynamics = np.zeros((len(cons_dynamics), 1))
    ub_dynamics = np.zeros((len(cons_dynamics), 1))

    lb_ineq = np.zeros((len(cons_ineq), 1)) - 1e9
    ub_ineq = np.zeros((len(cons_ineq), 1))

    ub_x = np.matlib.repmat(state_ub, N + 1, 1)
    lb_x = np.matlib.repmat(state_lb, N + 1, 1)
    ub_u = np.matlib.repmat(ctrl_ub, N, 1)
    lb_u = np.matlib.repmat(ctrl_lb, N, 1)
    ub_z = np.matlib.repmat(aux_ub, N, 1)
    lb_z = np.matlib.repmat(aux_lb, N, 1)

    lb_var = np.concatenate((lb_u.reshape((Dim_ctrl * N, 1)), 
                             lb_x.reshape((Dim_state * (N+1), 1)),
                             lb_z.reshape((Dim_aux * N, 1))
                             ))

    ub_var = np.concatenate((ub_u.reshape((Dim_ctrl * N, 1)), 
                             ub_x.reshape((Dim_state * (N+1), 1)),
                             ub_z.reshape((Dim_aux * N, 1))
                             ))

    vars_NLP   = ca.vertcat(u.reshape((Dim_ctrl * N, 1)), x.reshape((Dim_state * (N+1), 1)), z.reshape((Dim_aux * N, 1)))
    cons_NLP = cons_dynamics + cons_ineq + cons_init
    cons_NLP = ca.vertcat(*cons_NLP)
    lb_cons = np.concatenate((lb_dynamics, lb_ineq, lb_init_cons))
    ub_cons = np.concatenate((ub_dynamics, ub_ineq, ub_init_cons))

    prob = {"x": vars_NLP, "p":p, "f": J, "g":cons_NLP}

    return prob, N, vars_NLP.shape[0], cons_NLP.shape[0], p.shape[0], lb_var, ub_var, lb_cons, ub_cons