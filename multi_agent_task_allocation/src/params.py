import numpy as np
import os
import Additionals

mode  = 'sim' # 'cf'

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
    magazine = [15,5,3,3,3,3,3,3,3][:drone_num]
    linear_velocity = 2.5
    # base = [ (1.5,-0.7,1), (1.5,0,1), (1.5,0.7,1),(-1,0.2,1), (-1,0.2,1)][:drone_num] # (x,y,z) -> same coords definds in launch file
    base = [(0,-0.6,1), (0,0,1), (0,0.6,1)][:drone_num]

# ------------------ Allocation 
k_init = 5 
threshold_factor = 0.8
# -------------------   safety
safety_distance_trajectory = 0.4
safety_distance_allocation = safety_distance_trajectory * 1.2
floor_safety_distance = 0.8

# ------------------- Trajectory
resolution = 0.05 #[m]
retreat_range = 0.7 #[m]
take_off_height = base[0][2]
break_trajectory_len_factor = 0.2
offset_dist_target = 0.1 # [m]
# -------------------- Targets

data_source = 'circle'   
if data_source == 'circle':
    targets_num_gen = 30
    t = np.linspace(0, 2*np.pi-2*np.pi/targets_num_gen, targets_num_gen)
    radius = 0.6
    depth = 2.4
    z_offset = radius + floor_safety_distance + 0.3
    targetpos = np.stack([depth*np.ones([targets_num_gen]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)
    targetpos -= np.array([offset_dist_target, 0 ,0])
elif data_source == 'dataset':
    targetpos = np.load(str(os.getcwd())+'/src/rotors_simulator/multi_agent_task_allocation/src/targets_arr.npy')
    targetpos -= np.array([offset_dist_target, 0 ,0])
elif data_source == 'salon':
    targetpos  = Additionals.grid_shape() 
    targetpos -= np.array([offset_dist_target, 0 ,0])

targets_num, _ = targetpos.shape
mean_x_targets_position = np.sum(targetpos[:,0]) / targets_num
span, limits = Additionals.get_span(targetpos, base)

# --------------------- General
sleep_time = 0.2
colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ----------------- Plotting
plot_path_scatter=0
plot_smooth_path_cont=1
plot_smooth_path_scatter=0
plot_block_volume=1
elvazim = [37, 175]
plot_constant_blocking_area = 0


