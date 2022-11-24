import numpy as np
import os
mode  = 'sim' # 'cf'
# ----------- attional functions
def grid_shape():
    n = 20
    z_col = np.linspace(1,2.2,4)
    y_row = np.linspace(-3,3,5)
    x = 3.5
    targets = []
    for z in z_col:
        for y in y_row:
            targets.append([x,y,z])
    return np.array(targets)   

def get_span():
    z_min, z_max = 0, max( max(targetpos[:,2]), max(np.array(base)[:,2]))
    z_span = (z_max - z_min) * 1.3
    y_min, y_max =  min(min(targetpos[:,1]) , min(np.array(base)[:,1]))  , max(max(targetpos[:,1]), max(np.array(base)[:,1]))
    y_span = (y_max - y_min) * 1.3
    x_min, x_max = min(0 , min(np.array(base)[:,0])), max(max(targetpos[:,0]), max(np.array(base)[:,0])) 
    x_span = (x_max - x_min) * 1.3  
    return [x_span, y_span, z_span], [x_min, x_max, y_min, y_max, z_min, z_max]


# -------------------- CF
uri1 = 'radio://0/80/2M/E7E7E7E7E1'
uri2 = 'radio://0/80/2M/E7E7E7E7E2'
uri3 = 'radio://0/80/2M/E7E7E7E7E3'
uri4 = 'radio://0/80/2M/E7E7E7E7E4'
uri_list = [uri3, uri1, uri4] # index 0- most right drone 

# --------------------- Drones 
# ------drone CF
if mode == 'cf':
    drone_num = len(uri_list)
    magazine = [3,3,3,3,3,3,3,3,3][:drone_num]
    linear_velocity= 1 
    base = [(0,-0.6,1), (0,0,1), (0,0.6,1)][:drone_num]# (x,y,z)   -> right to left order

#-----drone sim
if mode == 'sim':
    drone_num = 3
    magazine = [3,3,3,3,3,3,3,3,3][:drone_num]
    linear_velocity = 2.5
    # base = [(1.5,0,1), (1.5,0.7,1), (1.5,-0.7,1), (-1,0.2,1), (-1,0.2,1)][:drone_num] # (x,y,z) -> same coords definds in launch file
    base = [(0,-0.6,1), (0,0,1), (0,0.6,1)][:drone_num]

# ------------------ Allocation 
k_init = 5 
threshold_factor = 0.8
# -------------------   safety
safety_distance_trajectory = 0.4
safety_distance_allocation = safety_distance_trajectory * 1.2
floor_safety_distance = 0.5

# ------------------- Trajectory
resolution = 0.05 #[m]
retreat_range = 0.7 #[m]
take_off_height = 1
break_trajectory_len_factor = 0.2
# -------------------- Targets

data_source = 'circle'   
if data_source == 'circle':
    targets_num_gen = 30
    t = np.linspace(0, 2*np.pi-2*np.pi/targets_num_gen, targets_num_gen)
    radius = 0.6
    depth = 3
    z_offset = radius + floor_safety_distance + 0.3
    targetpos = np.stack([depth*np.ones([targets_num_gen]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)
elif data_source == 'dataset':
    targetpos = np.load(str(os.getcwd())+'/src/rotors_simulator/multi_agent_task_allocation/src/targets_arr.npy')
elif data_source == 'salon':
    targetpos  = grid_shape()

targets_num, _ = targetpos.shape
mean_x_targets_position = np.sum(targetpos[:,0]) / targets_num
span, limits = get_span()
# --------------------- General
sleep_time = 0.3
colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ----------------- Plotting
plot_path_scatter=0
plot_smooth_path_cont=1
plot_smooth_path_scatter=0
plot_block_volume=1
elvazim = [37, 175]
plot_constant_blocking_area = 1


