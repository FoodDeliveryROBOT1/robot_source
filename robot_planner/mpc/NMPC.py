import casadi as ca
import numpy as np
import time
import matplotlib.pyplot as plt


def shift_movement(T, t0, x0, u, x_n, f):
    f_value = f(x0, u[0])
    st = x0 + T*f_value
    t = t0 + T
    u_end = np.concatenate((u[1:], u[-1:]))
    x_n = np.concatenate((x_n[1:], x_n[-1:]))
    return t, st, u_end, x_n

def desired_command_and_trajectory(t, T, x0_: np.array, rx, ry, N_):
    # initial state / last state
    x_ = np.zeros((N_+1, 3))
    x_[0] = x0_
    u_ = np.zeros((N_, 2))
    # states for the next N_ trajectories
    for i in range(N_):
        t_predict = t + T*i
        if t_predict < len(rx):
            x_ref_ = rx[int(t_predict)]
            y_ref_ = ry[int(t_predict)]
            theta_ref_ = np.arctan2(ry[int(t_predict)] - ry[int(t_predict)-1], rx[int(t_predict)] - rx[int(t_predict)-1])
            v_ref_ = 0.0      # Adjust linear and angular velocities as needed
            omega_ref_ = 0.0
        else:
            # Assuming the last point repeats
            x_ref_ = rx[-1]
            y_ref_ = ry[-1]
            theta_ref_ = np.arctan2(ry[-1] - ry[-2], rx[-1] - rx[-2])
            v_ref_ = 0.0
            omega_ref_ = 0.0
        x_[i+1] = np.array([x_ref_, y_ref_, theta_ref_])
        u_[i] = np.array([v_ref_, omega_ref_])
    # return pose and command
    return x_, u_



def prediction_state(x0, u, T, N):
    # define predition horizon function
    states = np.zeros((N+1, 3))
    states[0, :] = x0
    for i in range(N):
        states[i+1, 0] = states[i, 0] + u[i, 0] * np.cos(states[i, 2]) * T
        states[i+1, 1] = states[i, 1] + u[i, 0] * np.sin(states[i, 2]) * T
        states[i+1, 2] = states[i, 2] + u[i, 1] * T
    return states

class mpc_controller:
    def __init__(self, T, N, r_dim, v_max, omega_max, Q, R):
        self.T = T
        self.N = N
        self.r_dim = r_dim
        self.v_max = v_max
        self.omega_max = omega_max
        self.Q = Q
        self.R = R
        self.next_states =np.zeros((N+1, 3))
        self.next_controls = np.zeros((N, 2))
        self.u0 = np.zeros((N, 2))

        self.setup_controller()

    def setup_controller(self):
        self.opti = ca.Opti()
        self.opt_states = self.opti.variable(self.N+1,3)
        x = self.opt_states[:, 0]
        y = self.opt_states[:, 1]
        theta = self.opt_states[:, 2]

        self.opt_controls = self.opti.variable(self.N, 2)
        v = self.opt_controls[:, 0]
        omega = self.opt_controls[:, 1]

        self.opt_u_ref = self.opti.parameter(self.N,2)
        self.opt_x_ref = self.opti.parameter(self.N+1, 3)

        # Create Model
        f = lambda x_, u_: ca.vertcat(*[u_[0]*ca.cos(x_[2]), u_[0]*ca.sin(x_[2]), u_[1]])
        self.f_np = lambda x_, u_: np.array([u_[0]*np.cos(x_[2]), u_[0]*np.sin(x_[2]), u_[1]])


        self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])
        for i in range(self.N):
            x_next = self.opt_states[i, :] + f(self.opt_states[i, :], self.opt_controls[i, :]).T * self.T
            self.opti.subject_to(self.opt_states[i+1, :] == x_next)


        obj = 0
        for i in range(self.N):
            state_error_ = self.opt_states[i, :] - self.opt_x_ref[i+1, :]
            control_error_ = self.opt_controls[i, :] - self.opt_u_ref[i, :]
            obj = obj + ca.mtimes([state_error_, self.Q, state_error_.T]) + ca.mtimes([control_error_, self.R, control_error_.T])
        self.opti.minimize(obj)


        self.opti.subject_to(self.opti.bounded(-ca.inf, x, ca.inf))
        self.opti.subject_to(self.opti.bounded(-ca.inf, y, ca.inf))
        self.opti.subject_to(self.opti.bounded(-np.pi, theta, np.pi))
        self.opti.subject_to(self.opti.bounded(0, v, self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.omega_max, omega, self.omega_max))

        self.opts_setting = {
            'ipopt.max_iter': 2000,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol':1e-8,
            'ipopt.acceptable_obj_change_tol':1e-6
        }

        self.opti.solver('ipopt', self.opts_setting)

    def solver(self, next_trajectories, next_controls):
        # Set parameter, here only update initial state of x (x0)
        self.opti.set_value(self.opt_x_ref, next_trajectories)
        self.opti.set_value(self.opt_u_ref, next_controls)
        
        # Provide the initial guess of the optimization target
        self.opti.set_initial(self.opt_states, self.next_states)
        self.opti.set_initial(self.opt_controls, self.u0)
        
        # Solve the problem
        sol = self.opti.solve()
        
        # Obtain the control input
        self.u0 = sol.value(self.opt_controls)
        self.next_states = sol.value(self.opt_states)
        return self.u0, self.next_states, self.f_np

