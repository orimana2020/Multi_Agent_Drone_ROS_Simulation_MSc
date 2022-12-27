import numpy as np
import matplotlib.pyplot as plt
import os
import random
from mpl_toolkits.mplot3d import Axes3D
from scipy import interpolate
from scipy.interpolate import griddata, Rbf
from  matplotlib.colors import LinearSegmentedColormap



# curr_dir = 'cflib/Ori_CF'
curr_dir = 'src/rotors_simulator'

def load_results(lps_dir, vicon_dir, fix_values,samples_num ,is_2d, threshold, nodes):
    pos_lps_arr = []
    pos_vicon_arr = []
    error_arr = []
    sigma_arr = []
    for i in range(1, samples_num+1):
        measure_num = str(i)
        pos_lps = np.load(lps_dir+measure_num+'.npy')
        pos_vicon = np.load(vicon_dir+measure_num+'.npy')
        pos_lps_avg = np.average(pos_lps, axis=0)
        error_ = np.abs(pos_vicon - pos_lps_avg)
        if is_2d:
            error = np.array([max(0.02, error_[0]-fix_values[0]),max(0.02, error_[1]-fix_values[1]), 0])
        else:
            error = np.array([max(0.02, error_[0]-fix_values[0]),max(0.02, error_[1]-fix_values[1]),max(0.02, error_[2]-fix_values[2])])
        error_size = np.linalg.norm(error, ord=2)
        sigma = np.std(pos_lps, axis=0)
        if error_size < threshold: #filter out threshold 
            pos_lps_arr.append(pos_lps_avg)
            pos_vicon_arr.append(pos_vicon)
            error_arr.append(error_size)
            sigma_arr.append(sigma)
            # mirror results
            if pos_vicon[1] > 0.3:
                mirror_point = np.array([pos_vicon[0], -pos_vicon[1], pos_vicon[2]])
                pos_vicon_arr.append(mirror_point)
                error_arr.append(error_size)
                sigma_arr.append(sigma)
    # rotate and translate 
    rot_mat = np.array([[np.cos(np.pi), -np.sin(np.pi), 0 ],[np.sin(np.pi), np.cos(np.pi), 0],[0,0,1]])
    pos_vicon_arr = np.array(pos_vicon_arr) 
    translate_x = 1.13
    pos_vicon_arr = np.transpose(rot_mat.dot( np.transpose(pos_vicon_arr))) + np.array([translate_x, 0, 0])
    nodes = np.transpose(rot_mat.dot( np.transpose(nodes))) + np.array([translate_x, 0, 0])
    pos_lps_arr = np.array(pos_lps_arr)
    error_arr = np.array(error_arr)
    sigma_arr = np.array(sigma_arr)   
    return pos_lps_arr, pos_vicon_arr, error_arr, sigma_arr, nodes


def plot_sampled_data(error_arr, threshold, title, pos_vicon_arr, nodes):
    factor = error_arr  / threshold #smaller- green
    colors = []
    for fac in factor:
        colors.append([fac, 1-fac,0])
    colors = np.array(colors)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pos_vicon_arr[:,0], pos_vicon_arr[:,1], pos_vicon_arr[:,2],c=colors)
    ax.scatter(nodes[:,0],nodes[:,1],nodes[:,2],c='blue', s=100)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(title+' - Sampeled Data')


def interpolate_3d(limits, resolution, pos_vicon_arr, error_arr):
    grid_x, grid_y, grid_z = np.mgrid[limits[0][0]:limits[0][1]:resolution, limits[1][0]:limits[1][1]:resolution, limits[2][0]:limits[2][1]:resolution ]
    interpolated_error = griddata(pos_vicon_arr, error_arr, (grid_x, grid_y, grid_z ), method='linear')
    rbf4 = Rbf(pos_vicon_arr[:,0], pos_vicon_arr[:,1], pos_vicon_arr[:,2],error_arr, function='linear')#, smooth=5)
    exterpolated_error = rbf4(grid_x, grid_y, grid_z)
    # merge
    interp_f = interpolated_error.flatten()
    extep_f = exterpolated_error.flatten()
    merged = np.zeros(interp_f.shape)
    for i in range(len(interp_f)):
        merged[i] = interp_f[i] if not np.isnan(interp_f[i]) else extep_f[i]
    merged = merged.reshape(interpolated_error.shape)
    return grid_x, grid_y, grid_z, interpolated_error ,exterpolated_error, merged  


def plot_interpolate(interpolated_error, threshold, grid_x, grid_y, grid_z, nodes,title):
    factor = interpolated_error  #/ threshold 
    colors = []
    for xi in range(factor.shape[0]):
        for yi in range(factor.shape[1]):
            for zi in range(factor.shape[2]):
                fac = factor[xi,yi, zi]
                colors.append(fac)
    colors = np.array(colors)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    cmap=LinearSegmentedColormap.from_list('rg',["lime","red"], N=256) 
    data=ax.scatter3D(grid_x, grid_y, grid_z,c=colors, cmap=cmap,vmin=0,vmax=threshold)
    cbar = fig.colorbar(data, ax = ax, shrink = 0.5, aspect = 5)
    cbar.set_label('Error in [m]')
    ax.scatter(nodes[:,0],nodes[:,1],nodes[:,2],c='blue', s=100)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(title + ' - Interpolated Data')
    
def plot_histogram(error_arr, title):
    fig = plt.figure()
    ax = fig.add_subplot('111')
    ax.hist(error_arr)
    ax.set_xlabel('Error [meter]')
    ax.set_title(title + ' Histogram')

def plot_difference(error1, error2, grid_x, grid_y, grid_z,nodes1, nodes2 ,difference_threshold=None, difference_ratio=None):
    diff = error2 - error1
    factor = diff  / np.nanmax(np.abs(diff))
    if difference_ratio:
        factor = error2 / error1
    colors = []
    for xi in range(factor.shape[0]):
        for yi in range(factor.shape[1]):
            for zi in range(factor.shape[2]):
                fac = factor[xi,yi, zi]
                if difference_threshold == None and difference_ratio==None:  
                    if fac >= 0: # error2>error1 -> show red
                        colors.append(np.array([fac, 0, 0,1]))
                    elif fac < 0 : # error2<error1 -> show green
                        colors.append( np.array([0,-fac, 0,1]))   
                    else: #in case fac = np.nan
                        colors.append(np.array([1,1, 1,0]))
                elif difference_ratio and not difference_threshold:
                    if fac <= 1 - difference_ratio:
                        colors.append(np.array([0, 1, 0,1]))
                    else:
                        colors.append(np.array([1,1, 1,0]))
                elif difference_threshold and not difference_ratio: 
                    if diff[xi,yi, zi] < 0 and np.abs(diff[xi,yi, zi]) >= difference_threshold:
                        colors.append( np.array([0,1, 0,1]))
                    else:
                        colors.append(np.array([1, 1, 1,0]))

    colors = np.array(colors)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(grid_x, grid_y, grid_z,c=colors)
    ax.scatter(nodes1[:,0],nodes1[:,1],nodes1[:,2],c='blue', s=100)
    ax.scatter(nodes2[:,0],nodes2[:,1],nodes2[:,2],c='yellow', s=100)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title('difference')

def plot_hovering(dir, is_deck, exp_num, cutoff_start, cutoff_end):
    if is_deck:
        raw_data = np.load(dir+'/vicon_hovering_with_flowdeck'+str(exp_num)+'.npy')
    else:
        raw_data = np.load(dir+'/vicon_hovering_without_flowdeck'+str(exp_num)+'.npy')
    data_len = len(raw_data)
    data_cut_off_start = int(cutoff_start*data_len)
    data_cut_off_end = int(cutoff_end*data_len)
    data = raw_data[data_cut_off_start:data_cut_off_end, :]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:,0],data[:,1],data[:,2]) 
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    if is_deck:
        ax.set_title('hovering with flowdeck')
    else:
        ax.set_title('hovering without flowdeck')



# --------------------------params ----------------------------------------------------
#general params
error_2d = True
threshold = 0.15
limits = [[-0.32, 3.5], [-1.5,1.5], [0.2,2]]
resolution = 0.05

# box config params:
samples_num_exp1 = 81
fix_values_exp1 = np.array([0.062,0.051,0.035]) * 0.5
nodes1 = np.array([[1.00	,1.30	,0.23],[1.12	,1.40,	2.23],[1.06	,-1.14	,0.18],[1.13	,-1.32	,2.17],[-0.98	,-1.15	,0.16],[-0.84,	-1.28,	2.17],[-0.94	,1.36,	0.21],[-0.78	,1.44	,2.23]])
optimal_lps_dir = curr_dir +'/multi_agent_task_allocation/posinioning_measurements_experiment/results/optimal_config/LPS_optimal/lps_static_pos_optimal_config_'
optimal_vicon_dir = curr_dir +'/multi_agent_task_allocation/posinioning_measurements_experiment/results/optimal_config/vicon_optimal/vicon_optimal_config_'
title_exp1 = 'Box Configuration'

# extended params
samples_num_exp2 = 87
fix_values_exp2 = np.array([0.062,0.051,0.035]) * 0.75
nodes2 =  np.array([[1.00	,1.30	,0.23],[1.12	,1.40,	2.23],[1.06	,-1.14	,0.18],[1.13	,-1.32	,2.17],[-1,	-1.14	,0.12],[-2.31	,-1.30	,2.17],[-0.96,	1.38	,0.18],[-2.33	,1.45	,2.22]])
extended_lps_dir = curr_dir +'/multi_agent_task_allocation/posinioning_measurements_experiment/results/extended_config/LPS_extended/lps_static_pos_changed_config_'
extended_vicon_dir =  curr_dir +'/multi_agent_task_allocation/posinioning_measurements_experiment/results/extended_config/vicon_extended/vicon_changed_config_'
title_exp2 = 'Extended Configuration'


# hovering
hovering_dir = curr_dir +'/multi_agent_task_allocation/posinioning_measurements_experiment/results/hovering'
#----------------------Analysis---------------------------------------------------

# box config
lps_1, vicon_1, error_1, sigma_1, nodes1 = load_results(lps_dir=optimal_lps_dir, vicon_dir=optimal_vicon_dir, fix_values=fix_values_exp1,samples_num=samples_num_exp1 ,is_2d=error_2d, threshold=threshold, nodes=nodes1)
plot_histogram(error_1, title_exp1)
plot_sampled_data(error_1, threshold, title_exp1, vicon_1, nodes1)
grid_x, grid_y, grid_z, interp_err_1 , extep_arr_1, merged_1 = interpolate_3d(limits, resolution, vicon_1, error_1)
plot_interpolate(merged_1, threshold, grid_x, grid_y, grid_z, nodes1, title_exp1)


# extended configuration
lps_2, vicon_2, error_2, sigma_2, nodes2 = load_results(lps_dir=extended_lps_dir, vicon_dir=extended_vicon_dir, fix_values=fix_values_exp2,samples_num=samples_num_exp2 ,is_2d=error_2d, threshold=threshold, nodes=nodes2)
plot_histogram(error_2, title_exp2)
plot_sampled_data(error_2, threshold, title_exp2, vicon_2, nodes2)
grid_x, grid_y, grid_z, interpo_err_2, extep_arr_2, merged_2 = interpolate_3d(limits, resolution, vicon_2, error_2)
plot_interpolate(merged_2, threshold, grid_x, grid_y, grid_z, nodes2, title_exp2)


# diffence
plot_difference(merged_1, merged_2, grid_x, grid_y, grid_z ,nodes1, nodes2, difference_threshold=None, difference_ratio = 0.5)
plot_histogram(merged_2.flatten() - merged_1.flatten() , title='diff')


# hovering
plot_hovering(hovering_dir, is_deck=True, exp_num=1, cutoff_start=0.3,cutoff_end=0.7)


# save error array to load to simulation
save = False
plotting = False

# print(np.int8(np.ceil(merged_1/0.05)))
# print(np.max(np.int8(np.ceil(merged_1/0.05))))
if save:
    if resolution != 0.05:
          raise Exception("resolution should fit the simulation!")
    np.save('error_arr_box_config', merged_1) # [x,y,z]
    np.save('error_arr_extended_config', merged_2) # [x,y,z]
if plotting:
    plt.show()