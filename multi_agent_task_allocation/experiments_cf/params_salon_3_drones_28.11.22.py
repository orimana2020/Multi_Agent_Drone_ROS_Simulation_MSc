import numpy as np
import os
# ----------------------------- addtional functions for params ----------------- #
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

def get_span(targetpos, base, resolution):
    z_min, z_max = 0, max( max(targetpos[:,2]), max(np.array(base)[:,2]))
    z_span = (z_max - z_min) * 1.3
    y_min, y_max =  min(min(targetpos[:,1]) , min(np.array(base)[:,1]))  , max(max(targetpos[:,1]), max(np.array(base)[:,1]))
    y_span = (y_max - y_min) * 1.3
    x_min, x_max = min(0 , min(np.array(base)[:,0])), max(max(targetpos[:,0]), max(np.array(base)[:,0])) 
    x_span = (x_max - x_min) * 1.3  
    return [x_span, y_span, z_span], [x_min, x_max, y_min, y_max, z_min, z_max] \
    ,  [round(x_min/resolution), round(x_max/resolution), round(y_min/resolution), round(y_max/resolution), round(z_min/resolution), round(z_max/resolution)]     
# ------------------------------------------------------------------------------ #

mode  = 'cf' # 'cf'

# -------------------- CF
uri1 = 'radio://0/80/2M/E7E7E7E7E1'
uri2 = 'radio://0/80/2M/E7E7E7E7E2'
uri3 = 'radio://0/80/2M/E7E7E7E7E3'
uri4 = 'radio://0/80/2M/E7E7E7E7E4'
uri_list = [uri1, uri2, uri3] # index 0- most right drone 

# --------------------- Drones 
# ------drone CF
if mode == 'cf':
    drone_num = len(uri_list)
    magazine = [3,3,3,3,3,3,3,3,3][:drone_num]
    linear_velocity= 0.5
    base = [(0,-0.6,1), (0,0,1), (0,0.6,1)][:drone_num]# (x,y,z)   -> right to left order

#-----drone sim
if mode == 'sim':
    drone_num = 3
    magazine = [3,3,3,3,3,3,3,3,3][:drone_num]
    linear_velocity = 2.5
    # base = [ (1.5,-0.7,1), (1.5,0,1), (1.5,0.7,1),(-1,0.2,1), (-1,0.2,1)][:drone_num] # (x,y,z) -> same coords definds in launch file
    base = [(0,-0.6,1), (0,0,1), (0,0.6,1)][:drone_num]
    uri_list = [[0]] * drone_num

# ------------------ Allocation 
k_init = 5 
threshold_factor = 0.8
uri_state_mat_sim = '/src/rotors_simulator/multi_agent_task_allocation/src'
uri_targetpos_cf = '/cflib/Ori_CF/multi_agent_task_allocation/src'
if mode == 'sim':
    uri_state_mat = uri_state_mat_sim
elif mode == 'cf':
    uri_state_mat = uri_targetpos_cf

# -------------------   safety
safety_distance_trajectory = 0.4
safety_distance_allocation = safety_distance_trajectory * 1.2
floor_safety_distance = 0.5

# ------------------- Trajectory
resolution = 0.05 #[m]
retreat_range = 0.7 #[m]
take_off_height = base[0][2]
break_trajectory_len_factor = 0.2
offset_x_dist_target = 0.1 # [m]
dist_to_goal = 0.2
segments_num = 15 # max = 30
points_in_smooth_params = segments_num + 1


# -------------------- Targets
uri_targetpos_sim = '/src/rotors_simulator/multi_agent_task_allocation/datasets/pear/offset_data/pear_fruitpos_close_1offset_4_0_0.npy'
# uri_targetpos_cf = '/src/rotors_simulator/src/multi_agent_task_allocation/peach/peach_fruitpos_close_1.npy'
if mode == 'sim':
    target_uri = uri_targetpos_sim
elif mode == 'cf':
    target_uri = uri_targetpos_cf

data_source = 'circle'   
if data_source == 'circle':
    targets_num_gen = 25
    t = np.linspace(0, 2*np.pi-2*np.pi/targets_num_gen, targets_num_gen)
    radius = 0.6
    depth = 2
    z_offset = radius + floor_safety_distance + 0.1
    targetpos = np.stack([depth*np.ones([targets_num_gen]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)
elif data_source == 'dataset':
    targetpos = np.load(str(os.getcwd()) + target_uri)
elif data_source == 'salon':
    targetpos  = grid_shape() 
  
  
targetpos -= np.array([offset_x_dist_target, 0 ,0]) #########################
targets_num, _ = targetpos.shape
mean_x_targets_position = np.sum(targetpos[:,0]) / targets_num
span, limits, limits_idx = get_span(targetpos, base, resolution)
print(limits_idx)


# --------------------- General
sleep_time = 0.3
colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ----------------- Plotting
plot_path_scatter=0
plot_smooth_path_cont=1
plot_smooth_path_scatter=0
plot_block_volume=1
plot_constant_blocking_area = 1
elvazim = [37, 175]

