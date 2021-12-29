from ilqr.dynamics import FiniteDiffDynamics
from ilqr.cost import QRCost
from mujoco_py import load_model_from_path,  MjSim, MjViewer
import math
import numpy as np
import time
from ilqr import iLQR

model = load_model_from_path('.mujoco/mujoco200/model/my_biped_mpcModel.xml')
model_sim = load_model_from_path('.mujoco/mujoco200/model/my_biped.xml')
sim = MjSim(model)
actual_sim = MjSim(model_sim)
viewer = MjViewer(actual_sim)


state_size = 19*2-1  # [position, velocity]
action_size = 12  
dt = 0.01  # Discrete time-step in seconds.

Qq = np.zeros([state_size])

Qq[2] = 1
Qq[19:25] = 1
Qq[7:19] = 0.01

Q=np.diag(Qq)

Q=Q*1000
R = 0.0001 * np.eye(action_size)


x_goal = np.zeros(state_size)
x_goal[2]=1.2
x_goal[19]=0
cost = QRCost(Q, R, Q_terminal=Q, x_goal=x_goal)
class dynamic():
    def __init__(self):
        self.state_size = 37
        self.action_size = 12
    def f(self,x,u,i):
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

    def f_x(self, x, u,i):
        e=1e-5
        f_=self.f(x,u,i)
        df = np.zeros([37,37])
        sim_state = sim.get_state()
        for i  in range(len(u)):
            sim.data.ctrl[i]=u[i]
        for j in range(len(x)):
            dx=x.copy()
            dx[j]+=e
            for i, pos in enumerate(dx[0:19]):
                sim_state.qpos[i]=pos
            for i, vel in enumerate(dx[19:37]):
                sim_state.qvel[i]=vel

            sim.set_state(sim_state)
            sim.step()
            s=sim.get_state()
            pos_=s.qpos
            vel_=s.qvel
            df[:,j]=np.append(pos_,vel_)
        pfpx=(df-np.tile(np.transpose([f_]),37))/e
        return pfpx

    def f_u(self, x, u, i):
        e=1e-5
        f_=self.f(x,u,i)
        df = np.zeros([37,12])
        sim_state = sim.get_state()
        for i, pos in enumerate(x[0:19]):
            sim_state.qpos[i]=pos
        for i, vel in enumerate(x[19:37]):
            sim_state.qvel[i]=vel
        sim.set_state(sim_state)

        for j in range(len(u)):
            du=u.copy()
            du[j]=u[j]+e
            for i  in range(len(du)):
                sim.data.ctrl[i]=du[i]
            sim.set_state(sim_state)
            sim.step()
            s=sim.get_state()
            pos_=s.qpos
            vel_=s.qvel
            df[:,j]=np.append(pos_,vel_)
        pfpu=(df-np.tile(np.transpose([f_]),12))/e
        return pfpu



dynamics=dynamic()

T0=time.perf_counter()

N = 20  # Number of time-steps in trajectory.

us=np.array([])
us_init = np.random.uniform(-0.01, 0.01, (N, action_size)) # Random initial action path.
while True:
    s=actual_sim.get_state()
    pos_=s.qpos
    vel_=s.qvel
    x0=np.append(pos_,vel_)
    if len(us)>0:
        us_init=np.vstack((us[1:,:],np.zeros(action_size)))
    runilqr = iLQR(dynamics, cost, N)
    xs, us = runilqr.fit(x0, us_init,n_iterations=10,tol=1e-6)
    dT=time.perf_counter()-T0
    T0=time.perf_counter()

    
    # sim.set_state(s)
    for j in range(2):
        for i  in range(action_size):
            actual_sim.data.ctrl[i]=us[j,i]
        actual_sim.step()
        viewer.render()

    print(dT)




# def f(x, u, i):
#     sim_state = sim.get_state()
#     for i, pos in enumerate(x[0:19]):
#         sim_state.qpos[i]=pos
#     for i, vel in enumerate(x[0:18]):
#         sim_state.qvel[i]=vel
#     sim.set_state(sim_state)
#     for i  in range(len(u)):
#         sim.data.ctrl[i]=u[i]

#     sim.step()
#     s=sim.get_state()
#     pos_=s.qpos
#     vel_=s.qvel
#     return np.append(pos_,vel_)
# dynamics = FiniteDiffDynamics(f, state_size, action_size)









# print(xs[-1,:])

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