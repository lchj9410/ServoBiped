from mujoco_py import load_model_from_path,  MjSim, MjViewer
import math
import time
import numpy as np
model = load_model_from_path('.mujoco/mujoco200/model/my_biped.xml')
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0

x0=np.zeros(37)
x0[2]=1.29
x0[3] = 0.999
x0[0]+=1
sim_state = sim.get_state()
for i, pos in enumerate(x0[0:19]):
    sim_state.qpos[i]=pos
for i, vel in enumerate(x0[19:37]):
    sim_state.qvel[i]=vel
sim.set_state(sim_state)
sim.step()
sim_state = sim.get_state()
print(sim_state)
while True:
    # sim.data.ctrl[0] = math.cos(t / 10.) 
    # sim.data.ctrl[1] = math.sin(t / 10.) 
    # t+=1
    # t0=time.perf_counter()

    # print(time.perf_counter()-t0)
    viewer.render()
