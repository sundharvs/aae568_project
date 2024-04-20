from casadi import *
import numpy as np
from utils import plot

opti = Opti()

dt = 0.1 # time between steps in seconds
N = 10 # number of control intervals
sim_time = 10

# Constants
m = 2.06430769 # mass [kg]
l = 0.246 # arm length [m]
I = DM([0.0217,0.0217,0.04])  # inertia
g = 9.8
cd = 0.0
c_tau  = 0.05 # thrust torque coeff.
beta = DM([0,0,0])

X = opti.variable(12,N+1)
U = opti.variable(4,N)
X0 = opti.parameter(12,1)
red_pos = opti.parameter(3,1)

opti.minimize(sumsqr(X[0:3,:] - red_pos))

def f(x, u):
    p = x[0:3]
    e = x[3:6]
    v = x[6:9]
    w = x[9:]

    f_T = u[0]+u[1]+u[2]+u[3]
    tau_phi = l/sqrt(2)*(u[1]+u[2]-u[0]-u[3])
    tau_theta = l/sqrt(2)*(u[0]+u[2]-u[1]-u[3])
    tau_psi = c_tau*(u[0]+u[1]-u[2]-u[3])

    x_dot = vertcat(v,w,
        -1/m*(cos(e[0])*sin(e[1])*cos(e[2]) + sin(e[0])*sin(e[2]))*f_T, # - beta[0]/m*v[0]
        -1/m*(cos(e[0])*sin(e[1])*sin(e[2]) - sin(e[0])*cos(e[2]))*f_T, # - beta[1]/m*v[1]
        -1/m*cos(e[0])*cos(e[1])*f_T + g, # - beta[2]/m*v[2]
        -(I[2]-I[1])/I[0]*w[1]*w[2] + tau_phi/I[0],
        -(I[0]-I[2])/I[1]*w[0]*w[2] + tau_theta/I[1],
        -(I[1]-I[0])/I[2]*w[0]*w[1] + tau_psi/I[2]
    )

    return x_dot

for k in range(N):
   # Runge-Kutta 4 integration
   k1 = f(X[:,k],         U[:,k])
   k2 = f(X[:,k]+dt/2*k1, U[:,k])
   k3 = f(X[:,k]+dt/2*k2, U[:,k])
   k4 = f(X[:,k]+dt*k3,   U[:,k])
   x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
   opti.subject_to(X[:,k+1]==x_next)

opti.subject_to(opti.bounded(0.193,U,8.5))
opti.subject_to(X[:,0] == X0)
opti.subject_to(opti.bounded(-1.22,X[3:5,:],1.22))
opti.subject_to(opti.bounded(-100, X[2,:], 0))

opts = {
    "print_time": 1,
    "error_on_fail": 1,
    "ipopt": {
        "print_level": 1,
        # "acceptable_tol": 1e-6,
        # 'acceptable_obj_change_tol': 1e-6
    }
}

opti.solver('ipopt', opts)
# opti.solver('sqpmethod',dict(qpsol='ipopt'))

M = opti.to_function('M',[X0, red_pos],[U[:,0], X[:,1]],['X0', 'red_pos'],['u_opt', 'x_next'])

# M.generate('codegen_demo',{"main": True})

# MPC loop
X_log = []
U_log = []

x0 = DM([7,2,0,0,0,0,0,0,0,0,0,0])
target = DM([7,2,-10])

x = x0

mpc_iter = 0

while ((norm_2(x[0:3]) > 0.1) and (mpc_iter*dt) < sim_time):
    [u,x_next] = M(x, target)

    U_log.append(u)
    X_log.append(x.elements())

    # Runge-Kutta 4 integration
    k1 = f(x,         u)
    k2 = f(x+dt/2*k1, u)
    k3 = f(x+dt/2*k2, u)
    k4 = f(x+dt*k3,   u)
    x = (x + dt/6*(k1+2*k2+2*k3+k4))
    
    mpc_iter+=1

t_history = np.linspace(0, dt*mpc_iter, mpc_iter)
x_history = np.array(X_log)
u_history = np.array(U_log)

plot(x_history, u_history, t_history)

M.save('quadrotor_nlp.casadi')
M = Function.load('quadrotor_nlp.casadi')