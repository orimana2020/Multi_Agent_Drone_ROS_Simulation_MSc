import numpy as np
import os
# ----------------------------- addtional functions for params ----------------- #
def get_span(targetpos, base, resolution):
    z_min, z_max = 0, max( max(targetpos[:,2]), max(np.array(base)[:,2]))
    z_max = z_max + 0.2 # add offset for c_space
    z_span = (z_max - z_min) 
    y_min, y_max =  min(min(targetpos[:,1]) , min(np.array(base)[:,1]))  , max(max(targetpos[:,1]), max(np.array(base)[:,1]))
    y_min  -= 0.5 # add offset for c_space
    y_max += 0.5 # add offset for c_space
    y_span = (y_max - y_min) 
    x_min, x_max = 0, max(max(targetpos[:,0]), max(np.array(base)[:,0])) 
    x_max += 0.3 # add offset for c_space
    x_span = (x_max - x_min) 
    span_m = [x_span, y_span, z_span]
    min_max_m = [x_min, x_max, y_min, y_max, z_min, z_max]
    min_max_idx = [round(x_min/resolution), round(x_max/resolution), round(y_min/resolution), round(y_max/resolution), round(z_min/resolution), round(z_max/resolution)]     
    print(min_max_m)
    print(min_max_idx)
    return span_m, min_max_m, min_max_idx

# ------------------------------------------------------------------------------ #

mode  = 'sim' # 'cf' / 'sim'

# -------------------- CF -----------------------#
uri1 = 'radio://0/80/2M/E7E7E7E7E1'
uri2 = 'radio://0/80/2M/E7E7E7E7E2'
uri3 = 'radio://0/80/2M/E7E7E7E7E3'
uri4 = 'radio://0/80/2M/E7E7E7E7E4'
uri_list = [uri1,uri3] # index 0- most right drone 

# --------------------- Drones --------------------#
# -----------Drone CF
if mode == 'cf':
    drone_num = len(uri_list)
    magazine = [3,3,3,3,3,3,3,3,3][:drone_num]
    linear_velocity = 1
    drone_size_m = 0.15 # [m]
    base = [(0.3,-0.6,1), (0.3,0,1), (0.3,0.6,1),(0.3,0.9,1)][:drone_num]# (x,y,z)   -> right to left order

#-----------Drone Sim
if mode == 'sim':
    drone_num = 3
    magazine = [5,5,5,3,3,3,3,3,3][:drone_num]
    linear_velocity = 1
    base = [(0.3,-0.7,1), (0.3,0,1), (0.3,0.7,1),(0.3,0.9,1)][:drone_num] # (x,y,z)   -> right to left order
    uri_list = [[0]] * drone_num
    drone_size_m = 0.2 # [m]


# ------------------ Allocation --------------------#
k_init = 5 
threshold_factor = 0.8
uri_state_mat_sim = '/src/rotors_simulator/multi_agent_task_allocation/src'
uri_targetpos_cf = '/cflib/Ori_CF/multi_agent_task_allocation/src'
if mode == 'sim':
    uri_state_mat = uri_state_mat_sim
elif mode == 'cf':
    uri_state_mat = uri_targetpos_cf

# -------------------   safety
safety_distance_trajectory = 0.4 # update error map experiment
safety_distance_allocation = safety_distance_trajectory * 1.2 # update error map experiment
DOWNWASH_AWARE = True
floor_safety_distance = 0.5 
min_battery_voltage = 3.2 
check_battery_interval_time = 7 #[sec]

# ------------------- Trajectory
resolution = 0.05 #[m]
retreat_range = 0.7 #[m]
take_off_height = base[0][2]
break_trajectory_len_factor = 0.15
offset_x_dist_target = 0.5 # [m]
segments_num = 8 # max = 30
points_in_smooth_params = segments_num + 1
LPS_positioning_error_m = np.load(str(os.getcwd())+ uri_state_mat + '/positioning_error_arr/error_arr_box_config.npy')
LPS_n_safety_vol = np.int8(np.ceil(LPS_positioning_error_m / resolution)) + np.int8(drone_size_m / resolution)
SIMULATE_LPS_ERROR = False

if mode == 'sim':
    dist_to_target = 0.05
    dist_to_base = 0.1
elif mode == 'cf':
    dist_to_target = 0.05
    dist_to_base = 0.1

# -------------------- Targets
uri_targetpos_sim = '/src/rotors_simulator/multi_agent_task_allocation/datasets/pear/offset_data/pear_fruitpos_close_1offset_4_0_0.npy'
# uri_targetpos_cf = '/src/rotors_simulator/src/multi_agent_task_allocation/peach/peach_fruitpos_close_1.npy'
if mode == 'sim':
    target_uri = uri_targetpos_sim
elif mode == 'cf':
    target_uri = uri_targetpos_cf

data_source = 'first_exp_cyrcle'
if data_source == 'circle':
    targets_num_gen = 35
    t = np.linspace(0, 2*np.pi-2*np.pi/targets_num_gen, targets_num_gen)
    radius = 0.6
    depth = 2
    z_offset = radius + floor_safety_distance + 0.1
    targetpos = np.stack([depth*np.ones([targets_num_gen]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)
elif data_source == 'dataset':
    targetpos = np.load(str(os.getcwd()) + target_uri)

elif data_source == 'first_exp_cyrcle':
    targetpos_raw = np.array([[2.01,1.06,1.25],[2.15,1.01,1.11],[2.17,1.08,1.01],[2.2,0.91,0.90],[2.22,0.8,0.84],[2.26,1.09,0.8],[2.27,0.98,0.59],[2.3,1.35,0.59],[2.3,0.89,0.49],[2.29,0.83,0.47],[2.36,1.08,0.36],[2.41,0.91,0.14],[2.12,0.6,1.14],[2.09,0.22,1.21],[2.15,0.34,1.03],[2.13,0.19,1.03],[2.17,0.09,0.91],[2.18,0.17,0.84],[2.21,0.23,0.75],[2.22,0.11,0.67],[2.26,0.35,0.59],[2.29,0.24,0.5],[2.33,0.22,0.32],[2.43,0.46,0.06],[2.46,0.04,-0.01],[2.19,-0.2,0.97],[2.3,-0.23,0.53],[2.44,-0.3,0.13],[2.2,-0.39,0.86],[2.29,-0.51,0.64],[2.33,-0.43,0.54],[2.09,-0.66,1.24],[2.15,-0.77,1.11],[2.19,-0.61,0.95],[2.23,-0.76,0.83],[2.3,-0.67,0.6],[2.42,-0.82,0.3],[2.44,-0.73,0.2]])
    targetpos_raw = targetpos_raw + np.array([0 ,0 ,0.85]) # correct trimble z offset

targetpos_x_off = targetpos_raw - np.array([offset_x_dist_target, 0 ,0]) 
targetpos = np.array([target for target in targetpos_x_off if target[2] > floor_safety_distance + resolution * 2])
# targetpos_max_x_diff = round(abs(max(targetpos[:,0]) - min(targetpos[:,0]) ), 2)
targets_num, _ = targetpos.shape
mean_x_targets_position = np.sum(targetpos[:,0]) / targets_num
span, limits, limits_idx = get_span(targetpos, base, resolution)

# --------------------- Safety 2
downwash_distance = np.array([[min(targetpos[:,0]), max(targetpos[:,0])], [0.25,0.25], [1.5,1.5]]) # [m] , also distance to avoid flowdeck disturbance

# --------------------- General
sleep_time = 0.05
colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ----------------- Plotting
plot_path_scatter = 0
plot_smooth_path_cont = 1
plot_smooth_path_scatter = 0
plot_block_volume = 1
plot_constant_blocking_area = 1
plot_block_volume_floor_m = 0
elvazim = [37, 175]

# --------------- Analysis -------------
counter = np.load("counter_analysis.npy")
np.save("counter_analysis", np.array(counter+1))
print(f'Task num: {counter}')

