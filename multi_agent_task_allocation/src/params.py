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
    return span_m, min_max_m, min_max_idx

# ------------------------------------------------------------------------------ #

mode  = 'sim' # 'cf' / 'sim'

# -------------------- CF -----------------------#
uri1 = 'radio://0/80/2M/E7E7E7E7E1'
uri2 = 'radio://0/80/2M/E7E7E7E7E2'
uri3 = 'radio://0/80/2M/E7E7E7E7E3'
uri4 = 'radio://0/80/2M/E7E7E7E7E4'
uri_list = [uri1,uri2,uri3,uri4] # index 0- most right drone 

# --------------------- Drones --------------------#
# -----------Drone CF
if mode == 'cf':
    drone_num = len(uri_list)
    magazine = [3,3,3,3,3,3,3,3][:drone_num]
    linear_velocity = 1
    drone_size_m = 0.2 # [m]
    # base = [(0.6,-0.7,1), (0.6,0,1), (0.6,0.7,1)][:drone_num]# (x,y,z)   -> right to left order
    base = [(0.6,-1,1), (0.6,-0.3,1), (0.6,0.3,1), (0.6,1,1)][:drone_num]# (x,y,z)   -> right to left order

    segments_num = 8

#-----------Drone Sim
if mode == 'sim':
    drone_num = 3
    magazine = [10,10,10,10,10,10,10,3,3,3,3][:drone_num]
    linear_velocity = 1
    base = [(0.1,-0.7,1), (0.1,0,1), (0.1,0.7,1),(0.3,0.9,1)][:drone_num] # (x,y,z)   -> right to left order
    uri_list = [[0]] * drone_num
    delta = 0.3
    drone_size_m = 0.25+delta # [m]
    segments_num = 15


# ------------------ Allocation --------------------#
k_init = 10
threshold_factor = 0.8
uri_state_mat_sim = '/src/rotors_simulator/multi_agent_task_allocation/src'
uri_targetpos_cf = '/Ori_CF/multi_agent_task_allocation/src'
if mode == 'sim':
    uri_state_mat = uri_state_mat_sim
elif mode == 'cf':
    uri_state_mat = uri_targetpos_cf

# -------------------   safety
DOWNWASH_AWARE = True
floor_safety_distance = 0.3 
min_battery_voltage = 3.2 

# ------------------- Trajectory

resolution = 0.05 #[m]
retreat_range = 0.7 #[m]
take_off_height = base[0][2]
break_trajectory_len_factor = 0.15
offset_x_dist_target = 0.4 # [m]
LPS_positioning_error_m = np.load(str(os.getcwd())+ uri_state_mat + '/positioning_error_arr/error_arr_box_config.npy')
LPS_n_safety_vol = np.int8(np.ceil(LPS_positioning_error_m / resolution)) + np.int8(drone_size_m / resolution)
SIMULATE_LPS_ERROR = False

if mode == 'sim':
    dist_to_target = 0.05
    dist_to_base = 0.1
elif mode == 'cf':
    dist_to_target = 0.1
    dist_to_base = 0.1

# -------------------- Targets
uri_targetpos_sim = '/src/rotors_simulator/multi_agent_task_allocation/datasets/experiment1/experiment1_targets.npy'

data_source = 'dataset' 

if data_source == 'circle':
    targets_num_gen = 5; t = np.linspace(0, 2*np.pi-2*np.pi/targets_num_gen, targets_num_gen); radius=0.6; depth=2.1;z_offset = radius + floor_safety_distance + 0.1;
    targetpos_raw = np.stack([depth*np.ones([targets_num_gen]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)

elif data_source == 'dataset':
    targetpos_raw = np.load(str(os.getcwd()) + uri_targetpos_sim)

elif data_source == 'cf_exp':
    targetpos_raw = np.array([[2.024,-0.885,1.318],[2.11,-0.871,1.003],[2.071,-0.600,1.246],[2.102,-0.681,1.079],[2.147,-0.734,0.934],[2.109,-0.484,1.044],[2.138,-0.580,0.956],[2.045,-0.208,1.252],[2.100,-0.301,1.059],[2.132,-0.295,0.967],[2.163,-0.386,0.869],[2.236,-0.457,0.577],[2.241,-0.641,0.584],[2.295,-0.435,0.435],[2.340,-0.997,0.358],[2.319,-0.363,0.327],[2.354,-0.440,0.241],[2.377,-0.647,0.169],[2.162,-0.230,0.838],[2.217,-0.252,0.647],[2.413,-0.340,-0.043],[1.982,0.141,1.378],[2.054,-0.037,1.143],[2.065,0.085,1.117],[2.082,0.083,1.007],[2.144,0.079,0.765],[2.224,0.030,0.475],[2.284,-0.121,0.350],[2.334,-0.115,0.171],[2.087,0.354,1.096],[2.150,0.209,0.894],[2.212,0.413,0.680],[2.218,0.235,0.575],[2.247,0.176,0.437],[2.266,0.230,0.369],[2.363,0.147,0.042],[2.058,0.687,1.239],[2.086,0.493,1.129],[2.127,0.643,0.989],[2.165,0.680,0.819],[2.193,0.485,0.690],[2.234,0.606,0.546],[2.295,0.616,0.329],[2.319,0.405,0.279],[2.385,0.579,0.062],[2.103,0.914,1.082],[2.127,0.992,0.950],[2.188,0.871,0.774],[2.202,1.008,0.734],[2.270,1.087,0.431],[2.287,0.812,0.323],[2.384,0.882,0.025]])
    targetpos_raw = targetpos_raw + np.array([0 ,0 ,0.8]) # correct trimble z offset

targetpos_x_off = targetpos_raw - np.array([offset_x_dist_target, 0, 0]) 
targetpos = np.array([target for target in targetpos_x_off if target[2] > floor_safety_distance + resolution * 2])
targets_num, _ = targetpos.shape
print(f'target num: {targets_num}')
mean_x_targets_position = np.sum(targetpos[:,0]) / targets_num
span, limits, limits_idx = get_span(targetpos, base, resolution)

# --------------------- Safety 2
downwash_distance = np.array([[min(targetpos[:,0]), max(targetpos[:,0])], [0.35+delta,0.35+delta], [1.5+delta,1.5+delta]]) # [m] , also distance to avoid flowdeck disturbance

# --------------------- General
sleep_time = 0.1
colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ----------------- Plotting
plot_path_scatter = 0
plot_smooth_path_cont = 1
plot_smooth_path_scatter = 0
plot_block_volume = 1
plot_constant_blocking_area = 0
plot_block_volume_floor_m = 0
plot_save = False
elvazim = [37, 175]

# LPS --------------

trible_floor_offset = np.array([0,0,0.86])
LPS_trible = np.array([[0.054,-1.747,-0.605],[-0.181,-1.787,1.307],[1.747,-1.709,-0.506],[1.829,-1.913,1.325],[-0.236,1.553,-0.593],[-0.028,1.711,1.335],[1.698,1.692,-0.515],[1.736,1.883,1.338]])
LPS_anchor_pos = LPS_trible + trible_floor_offset



# --------------- Analysis -------------
# counter = np.load("counter_analysis.npy")
# file_name = 'task_k_'+str(k_init)+'_threshold_'+str(threshold_factor)+'_3'
file_name = 'ros_sim_funnel2'
print(file_name)
# np.save("counter_analysis", np.array(counter+1))
# print(f'Task num: {counter}')

