from ilqr.dynamics import FiniteDiffDynamics
from ilqr.cost import QRCost
from mujoco_py import load_model_from_path,  MjSim, MjViewer
import math
import numpy as np
import time
from ilqr import iLQR

model = load_model_from_path('.mujoco/mujoco200/model/my_biped_mpcModel.xml')
sim = MjSim(model)



state_size = 19*2-1  # [position, velocity]
action_size = 12  
dt = 0.01  # Discrete time-step in seconds.

Q = np.zeros([state_size,state_size])
Q[2,2] = 100
R = 0.001 * np.eye(action_size)


# State goal is set to a position of 1 m with no velocity.
x_goal = np.zeros(state_size)
x_goal[2]=1.1
# NOTE: This is instantaneous and completely accurate.
cost = QRCost(Q, R, Q_terminal=Q, x_goal=x_goal)

def f(x, u, i):
    sim_state = sim.get_state()
    for i, pos in enumerate(x[0:19]):
        sim_state.qpos[i]=pos
    for i, vel in enumerate(x[19:37]):
        sim_state.qvel[i]=vel
    sim.set_state(sim_state)
    for i  in range(len(u)):
        sim.data.ctrl[i]=u[i]

    sim.step()
    s=sim.get_state()
    pos_=s.qpos
    vel_=s.qvel
    return np.append(pos_,vel_)
dynamics = FiniteDiffDynamics(f, state_size, action_size)



T0=time.perf_counter()

N = 30  # Number of time-steps in trajectory.
x0=np.zeros(state_size)
x0[2]=2
u0=np.zeros(action_size)
us_init = np.random.uniform(-1, 1, (N, action_size)) # Random initial action path.
runilqr = iLQR(dynamics, cost, N)
xs, us = runilqr.fit(x0, us_init,n_iterations=10)


dT=time.perf_counter()-T0

print(dT)


print(xs[-1,:])

### test
# state_size = 2  # [position, velocity]
# action_size = 1  # [force]

# dt = 0.01  # Discrete time-step in seconds.
# m = 1.0  # Mass in kg.
# alpha = 0.1  # Friction coefficient.

# def f(x, u, i):
#     """Dynamics model function.

#     Args:
#         x: State vector [state_size].
#         u: Control vector [action_size].
#         i: Current time step.

#     Returns:
#         Next state vector [state_size].
#     """
#     [x, x_dot] = x
#     [F] = u

#     # Acceleration.
#     x_dot_dot = x_dot * (1 - alpha * dt / m) + F * dt / m

#     return np.array([
#         x + x_dot * dt,
#         x_dot + x_dot_dot * dt,
#     ])

# # NOTE: Unlike with AutoDiffDynamics, this is instantaneous, but will not be
# # as accurate.
# dynamics = FiniteDiffDynamics(f, state_size, action_size)
# curr_x = np.array([1.0, 2.0])
# curr_u = np.array([0.0])
# i = 0
# dynamics.f(curr_x, curr_u, i)
# dynamics.f_x(curr_x, curr_u, i)
# dynamics.f_u(curr_x, curr_u, i)