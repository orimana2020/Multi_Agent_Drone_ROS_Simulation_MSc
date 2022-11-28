import numpy as np
import os

arr = np.load(os.getcwd()+'/src/rotors_simulator/multi_agent_task_allocation/datasets/pear/offset_data/pear_fruitpos_close_1offset_4_0_0.npy')
print(arr)
print(min(arr[:,2]))
print(max(arr[:,2]))
x,y,z,w = None