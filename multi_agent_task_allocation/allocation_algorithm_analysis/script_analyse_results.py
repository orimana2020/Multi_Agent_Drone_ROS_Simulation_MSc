import numpy as np
import matplotlib.pyplot as plt
import os
from mpl_toolkits.mplot3d import Axes3D


colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ------------- experiment_parmas -----------------
mode = 'sim'
k_init = 10
threshold_factor = 0.9
i=0

k_lst = [2,3,4,5,6,7,8,9,10,11,12,13]
threshold_lst = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
samples = 5
# threshold_lst = [0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95]
# ------ what to show
analysis = 0
restore = 0
restore_history = 0
show_cost = 0
show_path = 0
compare_k_threshold=0
cost_as_k = 0
cost_as_threshold = 1



if mode == 'sim':
    url = str(os.getcwd()) +'/src/rotors_simulator/multi_agent_task_allocation/experiments_no_vision/300tar_random/'
    # url = ''
    # data = np.load('ros_sim_data_cp_1.npy', allow_pickle=True)
    data = np.load(url + 'no_visual_experiment_data_k_'+str(k_init)+'_thresh_'+str(threshold_factor)+'_'+str(i)+".npy", allow_pickle=True)


data = data.item()
general_data, drone_data = data['general_data'],  data['drone_data']

# Allocatio
allocation_history = general_data['allocation_history']
min_dist = allocation_history['min_dist']
threshold_up = allocation_history['threshold_up']
threshold_low = allocation_history['threshold_low']
combination = allocation_history['combination']
drone_num = allocation_history['drone_num']
kmeans = allocation_history['is_kmeans']
targetpos = general_data['targets_position']
next_diff = allocation_history['next_diff']
travel_dist = allocation_history['travel_dist'] 
cost = allocation_history['min_cost']
paths = allocation_history['path'] # drone_idx, start_title, goal_title, waypoints
initial_drone_num = general_data['initial_drone_num']
idx = np.array(range(0,len(min_dist)))
#check drone num changed idx
current_drone_num = general_data['initial_drone_num']
drone_change_idx  = []
for i, drone in enumerate(drone_num):
    if drone != current_drone_num:
        drone_change_idx.append(i)
        current_drone_num = drone
drone_change_idx = [x-0.5 for x in drone_change_idx]
# check kmeans
kmeans_idx = [idx-0.5 for idx in range(len(kmeans)) if kmeans[idx]==1] 


# # ------------- analysis ------------------

if analysis:
    plt.ioff()
    # Allocation
    fig1 = plt.figure()
    fig1.suptitle(f'k: {k_init}, threshold factor: {threshold_factor}, median cost: {round(np.median(cost),2)}, task_time: {round(general_data["total_task_time"], 2)} [sec]')
    ax1 = fig1.add_subplot('111')
    ax1.scatter(idx, min_dist,c='blue', label='min_dist',s=2)
    ax1.scatter(idx, threshold_low, c='green', label='Threshold',s=2)
    ax1.scatter(idx, threshold_up, c='green', label='Threshold',s=2)
    ax1.vlines(x=drone_change_idx, ymin=0, ymax=max(min_dist), colors='purple', ls='--', lw=1, label='Drone Num changed')
    ax1.vlines(x=kmeans_idx, ymin=0, ymax=max(min_dist), colors='yellow', ls='--', lw=0.5, label='KMEANS')
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Distance [m]")
    ax1.set_title('Allocation Performance')
    ax1.legend()
    plt.show()


# ----- restore
if restore:
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    limits = [[min(targetpos[:,0]), max(targetpos[:,0])],[min(targetpos[:,1]), max(targetpos[:,1])],[min(targetpos[:,2]), max(targetpos[:,2])]]
    for comb in combination:
        ax.axes.clear()
        ax.scatter3D(targetpos[:,0],targetpos[:,1],targetpos[:,2],  s= 10, c='k',alpha=1, depthshade=False)
        for j in range(len(comb)):
            ax.scatter3D(targetpos[comb[j],0],targetpos[comb[j],1],targetpos[comb[j],2],  s= 70, c=colors[j],alpha=1, depthshade=False)
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim((limits[0][0]-2,limits[0][1]))
        ax.set_ylim(limits[1])
        ax.set_zlim(limits[2])
        fig.canvas.flush_events()
        plt.pause(0.1)

if restore_history:
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    limits = [[min(targetpos[:,0]), max(targetpos[:,0])],[min(targetpos[:,1]), max(targetpos[:,1])],[min(targetpos[:,2]), max(targetpos[:,2])]]
    history = [[] for _ in range(initial_drone_num)]
    for comb in combination:
        ax.axes.clear()
        ax.scatter3D(targetpos[:,0],targetpos[:,1],targetpos[:,2],  s= 10, c='k',alpha=1, depthshade=False)
        for j in range(len(comb)):
            ax.scatter3D(targetpos[comb[j],0],targetpos[comb[j],1],targetpos[comb[j],2],  s= 120, c=colors[j],alpha=1, depthshade=False)
            history[j].append(comb[j])
        for j in range(initial_drone_num):
            ax.scatter3D(targetpos[history[j],0], targetpos[history[j],1], targetpos[history[j],2],  s= 30, c=colors[j],alpha=1, depthshade=False)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim((limits[0][0]-2,limits[0][1]))
        ax.set_ylim(limits[1])
        ax.set_zlim(limits[2])
        fig.canvas.flush_events()
        plt.pause(0.05)
            
if show_cost:
    plt.ioff()
    # Allocation
    fig3 = plt.figure()
    # fig3.suptitle(f'k: {k_init}, threshold factor: {threshold_factor}, average: {round(np.average(min_dist),2)}, task_time: {round(general_data["total_task_time"], 2)} [sec]')
    ax3= fig3.add_subplot('111')
    idx = np.array(range(0,len(next_diff)))
    ax3.scatter(idx, next_diff,c='blue', label='next_diff',s=10)
    ax3.scatter(idx, travel_dist, c='green', label='travel_dist',s=10)
    ax3.scatter(idx, cost, c='red', label='travel_dist',s=10)
    ax3.set_xlabel("Iteration")
    ax3.set_ylabel("cost")
    ax3.set_title('cost')
    ax3.legend()    
    plt.show()

if show_path:
    plt.ioff()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter3D(targetpos[:,0],targetpos[:,1],targetpos[:,2],  s= 10, c='k',alpha=1, depthshade=False)
    for path in paths:
        drone_idx, start_title, goal_title, waypoints = path
        if start_title == 'base' and goal_title == 'target':
            ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2], c=colors[drone_idx],alpha=1)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

if compare_k_threshold:
    exp_data = np.zeros([len(k_lst)*len(threshold_lst),3], dtype=float)
    idx = 0
    for k in k_lst:
        for thersh in threshold_lst:
            median_cost_lst = []
            for i in range(3):
                data = np.load(url + 'no_visual_experiment_data_k_'+str(k)+'_thresh_'+str(thersh)+'_'+str(i)+".npy", allow_pickle=True)
                data = data.item()
                general_data, drone_data = data['general_data'],  data['drone_data']
                allocation_history = general_data['allocation_history']
                median_cost_lst.append(np.average(allocation_history['min_cost']))
            average_cost = np.average(median_cost_lst)
            exp_data[idx,:] = [k, thersh, average_cost ]
            idx += 1
    k = exp_data[:,0]
    threshold  = exp_data[:,1]
    median_cost = exp_data[:,2]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter3D(k, threshold, median_cost)
    ax.set_xlabel('k')
    ax.set_ylabel('threshold')
    ax.set_zlabel('cost') 
    plt.show()






            
if cost_as_k:
    exp_data = np.zeros([len(k_lst),3], dtype=float)
    idx = 0
    for k in k_lst:
        cost_lst = []
        for thersh in threshold_lst:
            std_lst = []
            cost_sub_lst = []
            for i in range(samples ):
                data = np.load(url + 'no_visual_experiment_data_k_'+str(k)+'_thresh_'+str(thersh)+'_'+str(i)+".npy", allow_pickle=True)
                data = data.item()
                general_data, drone_data = data['general_data'],  data['drone_data']
                allocation_history = general_data['allocation_history']
                cost_lst.append(np.average(allocation_history['min_cost']))
                cost_sub_lst.append(np.average(allocation_history['min_cost']))
            std_lst.append(np.std(cost_sub_lst))
        average_std = np.average(std_lst)
        average_cost = np.average(cost_lst)
        exp_data[idx,:] = [k, average_cost,average_std]
        idx +=1
    k = exp_data[:,0]
    average_cost = exp_data[:,1]
    average_std = exp_data[:,2]
    fig = plt.figure()
    ax = fig.add_subplot('111')
    ax.scatter(k,average_cost)
    ax.errorbar(k,average_cost, average_std)
    ax.set_title('cost as function of k')
    ax.set_xlabel('k')
    ax.set_ylabel('cost')
    plt.show()   


if cost_as_threshold:
    exp_data = np.zeros([len(threshold_lst),3], dtype=float)
    idx = 0
    for thersh in threshold_lst:
        average_cost_lst = []
        for k in k_lst:
            std_lst = []
            cost_sub_lst = []
            for i in range(samples ):
                data = np.load(url + 'no_visual_experiment_data_k_'+str(k)+'_thresh_'+str(thersh)+'_'+str(i)+".npy", allow_pickle=True)
                data = data.item()
                general_data, drone_data = data['general_data'],  data['drone_data']
                allocation_history = general_data['allocation_history']
                average_cost_lst.append(np.average(allocation_history['min_cost']))
                cost_sub_lst.append(np.average(allocation_history['min_cost']))
            std_lst.append(np.std(cost_sub_lst))
        average_std = np.average(std_lst)
        average_cost = np.average(average_cost_lst)
        exp_data[idx,:] = [thersh, average_cost, average_std]
        idx +=1
    thresh = exp_data[:,0]
    average_cost = exp_data[:,1]
    average_std = exp_data[:,2]
    fig = plt.figure()
    ax = fig.add_subplot('111')
    ax.scatter(thresh, average_cost)
    ax.errorbar(thresh ,average_cost, average_std)
    ax.set_xlabel('thersh')
    ax.set_ylabel('cost')
    ax.set_title('cost as function of threshold')
    plt.show()   