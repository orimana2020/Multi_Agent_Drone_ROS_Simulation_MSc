import numpy as np
import matplotlib.pyplot as plt
import os
from mpl_toolkits.mplot3d import Axes3D


colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ------------- experiment_parmas -----------------
k_init = 10
threshold_factor = 0.7
i=0
fig_save = False
# ------ what to show
analysis = 0
restore = 0
restore_history = 0
show_cost = 0
show_path = 0
compare_k_threshold=0
cost_as_k = 0
cost_as_threshold = 0

#  dataset:
random300 = 0
dataset178_allocation = 0
rossim178 = 1
relative_distance = 1

# show in z axis
show_average_cost = 0
show_median_cost = 1
show_median_min_dist = 0
show_average_min_dist = 0
show_variance_min_dist=0
show_variance_cost =0
show_average_min_dist_to_variance = 0
show_average_min_cost_to_variance = 0

if random300:
    """ 300 random"""
    url = str(os.getcwd()) +'/src/rotors_simulator/multi_agent_task_allocation/experiments_no_vision/300tar_random2/'
    fig_title = '300 Randomly Placed Targets'
    data = np.load(url + 'no_visual_experiment_data_k_'+str(k_init)+'_thresh_'+str(threshold_factor)+'_'+str(i)+".npy", allow_pickle=True)
    k_lst = [2,3,4,5,6,7,8,9,10,11,12,13]
    threshold_lst = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
    samples = 5

if dataset178_allocation:
    """178 dataset allocation algorithm test"""
    url = str(os.getcwd()) +'/src/rotors_simulator/multi_agent_task_allocation/experiments_no_vision/dataset_no_vis/'
    fig_title = '178 Dataset Targets'
    data = np.load(url + 'no_visual_experiment_data_k_'+str(k_init)+'_thresh_'+str(threshold_factor)+'_'+str(i)+".npy", allow_pickle=True)
    k_lst = [2,3,4,5,6,7,8,9,10,11,12,13]
    threshold_lst = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
    samples = 5

if rossim178:
    """178 dataset ros sim"""
    url = str(os.getcwd()) +'/src/rotors_simulator/multi_agent_task_allocation/experiment_sim/experiment_1/exp4/'
    fig_title = '178 Dataset Targets - ROS Simulation'
    data = np.load(url + 'ros_sim_data_k_'+str(k_init)+'_thresh_'+str(threshold_factor)+".npy", allow_pickle=True)
    k_lst = [2,3,4,5,7,9,10,13]
    threshold_lst = [0.1,0.3,0.4,0.5,0.7,0.8,0.9]
    samples = 0



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
    fig1 = plt.figure()
    ax1 = fig1.add_subplot('111')
    ax1.scatter(idx[:-2], min_dist[:-2], c='blue', label='Minimum Distance',s=3)
    ax1.scatter(idx[:-2], threshold_low[:-2], c='green', label='Threshold',s=2)
    ax1.scatter(idx[:-2], threshold_up[:-2], c='green',s=2)
    ax1.vlines(x=kmeans_idx, ymin=0, ymax=max(min_dist[:-2]), colors='fuchsia', ls='--', lw=0.5, label='Kmeans')
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Minimum Distance(m)")
    ax1.set_title(f'{fig_title} \n k:{k_init}, Threshold Factor:{threshold_factor},\n Average Minimum Distance:{round(np.median(min_dist),2)} , Average Cost:{round(np.average(cost),2)}')
    ax1.legend(loc='lower left')
    ax1.grid(axis='y')
    if fig_save:
        fig1.savefig(fname='k_'+str(k_init)+'_threshold_'+str(threshold_factor)+'.png')
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
    fig3 = plt.figure()
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
        if start_title == 'target' and goal_title == 'base':
            ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2], c=colors[drone_idx],alpha=1)
        # if start_title == 'target' and goal_title == 'target':
        #     ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2], c=colors[drone_idx],alpha=1) 
    ax.set_title('Trajectories')       
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

if samples > 0:
    if compare_k_threshold:
        exp_data = np.zeros([len(k_lst)*len(threshold_lst),3], dtype=float)
        idx = 0
        for k in k_lst:
            for thersh in threshold_lst:
                median_cost_lst = []
                for i in range(samples):
                    data = np.load(url + 'no_visual_experiment_data_k_'+str(k)+'_thresh_'+str(thersh)+'_'+str(i)+".npy", allow_pickle=True)
                    data = data.item()
                    general_data, drone_data = data['general_data'],  data['drone_data']
                    allocation_history = general_data['allocation_history']
                    if show_median_cost:
                        median_cost_lst.append(np.median(allocation_history['min_cost']))
                    elif show_average_cost:
                        median_cost_lst.append(np.average(allocation_history['min_cost']))
                    elif show_median_min_dist:
                        median_cost_lst.append(np.median(allocation_history['min_dist']))
                    elif show_average_min_dist:
                        median_cost_lst.append(np.average(allocation_history['min_dist']))
                    elif show_variance_min_dist:
                        median_cost_lst.append(np.var(allocation_history['min_dist']))
                    elif show_variance_cost:
                        median_cost_lst.append(np.var(allocation_history['min_cost']))
                    elif show_average_min_dist_to_variance:
                        median_cost_lst.append(np.average(allocation_history['min_dist'])  /  np.var(allocation_history['min_dist']))
                    elif show_average_min_cost_to_variance:
                        median_cost_lst.append(np.average(allocation_history['min_cost'])  *  np.var(allocation_history['min_cost']))

                average_cost = np.average(median_cost_lst)
                exp_data[idx,:] = [k, thersh, average_cost ]
                idx += 1
        k = exp_data[:,0]
        threshold  = exp_data[:,1]
        average_cost = exp_data[:,2]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter3D(k, threshold, average_cost)
        ax.set_xlabel('K')
        ax.set_ylabel('Threshold')
        ax.set_zlabel('Cost') 
        ax.set_title(f'{fig_title} \n Cost as Function of Threshold and K')
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
                    if show_average_cost:
                        cost_lst.append(np.average(allocation_history['min_cost']))
                        cost_sub_lst.append(np.average(allocation_history['min_cost']))
                    elif show_median_cost:
                        cost_lst.append(np.median(allocation_history['min_cost']))
                        cost_sub_lst.append(np.median(allocation_history['min_cost']))
                    elif show_average_min_dist:
                        cost_lst.append(np.average(allocation_history['min_dist']))
                        cost_sub_lst.append(np.average(allocation_history['min_dist']))
                    elif show_median_min_dist:
                        cost_lst.append(np.median(allocation_history['min_dist']))
                        cost_sub_lst.append(np.median(allocation_history['min_dist']))
                    
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
        fig.suptitle(fig_title)
        ax.scatter(k,average_cost)
        ax.errorbar(k,average_cost, average_std,capsize = 3, fmt="" ,ecolor='k')
        ax.set_title('Cost as Function of K')
        ax.set_xlabel('K')
        ax.set_ylabel('Cost')
        ax.grid(axis='y')
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
                    if show_average_cost:
                        cost_lst.append(np.average(allocation_history['min_cost']))
                        cost_sub_lst.append(np.average(allocation_history['min_cost']))
                    elif show_median_cost:
                        cost_lst.append(np.median(allocation_history['min_cost']))
                        cost_sub_lst.append(np.median(allocation_history['min_cost']))
                    elif show_average_min_dist:
                        cost_lst.append(np.average(allocation_history['min_dist']))
                        cost_sub_lst.append(np.average(allocation_history['min_dist']))
                    elif show_median_min_dist:
                        cost_lst.append(np.median(allocation_history['min_dist']))
                        cost_sub_lst.append(np.median(allocation_history['min_dist']))
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
        ax.errorbar(thresh ,average_cost, average_std,capsize = 3, fmt="" ,ecolor='k')
        ax.set_xlabel('Treshold')
        ax.set_ylabel('Cost')
        ax.set_title(f'{fig_title} \n Cost as Function of Threshold')
        ax.grid(axis='y')
        plt.show()   












# -------------------------- ros sim analysis ---------------------

if samples == 0:
    if compare_k_threshold:
        exp_data = np.zeros([len(k_lst)*len(threshold_lst),3], dtype=float)
        idx = 0
        for k in k_lst:
            for thersh in threshold_lst:
                data = np.load(url + 'ros_sim_data_k_'+str(k)+'_thresh_'+str(thersh)+".npy", allow_pickle=True)
                data = data.item()
                general_data, drone_data = data['general_data'],  data['drone_data']
                allocation_history = general_data['allocation_history']
                if show_median_cost:
                    average_cost = (np.median(allocation_history['min_cost']))
                elif show_average_cost:
                    average_cost = (np.average(allocation_history['min_cost']))
                elif show_median_min_dist:
                    average_cost = (np.median(allocation_history['min_dist']))
                elif show_average_min_dist:
                    average_cost = (np.average(allocation_history['min_dist']))
                elif show_variance_min_dist:
                    average_cost = (np.var(allocation_history['min_dist']))
                elif show_variance_cost:
                    average_cost = (np.var(allocation_history['min_cost']))
                elif show_average_min_dist_to_variance:
                    average_cost = (np.average(allocation_history['min_dist'])  /  np.var(allocation_history['min_dist']))
                elif show_average_min_cost_to_variance:
                    average_cost = (np.average(allocation_history['min_cost'])  *  np.var(allocation_history['min_cost']))

                exp_data[idx,:] = [k, thersh, average_cost ]
                idx += 1
        k = exp_data[:,0]
        threshold  = exp_data[:,1]
        average_cost = exp_data[:,2]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter3D(k, threshold, average_cost)
        ax.set_xlabel('K')
        ax.set_ylabel('Threshold')
        ax.set_zlabel('Cost') 
        ax.set_title(f'{fig_title} \n Cost as Function of Threshold and K')
        plt.show()

                
    if cost_as_k:
        exp_data = np.zeros([len(k_lst),2], dtype=float)
        idx = 0
        for k in k_lst:
            cost_lst = []
            for thersh in threshold_lst:
                std_lst = []
                data = np.load(url + 'ros_sim_data_k_'+str(k)+'_thresh_'+str(thersh)+".npy", allow_pickle=True)
                data = data.item()
                general_data, drone_data = data['general_data'],  data['drone_data']
                allocation_history = general_data['allocation_history']
                cost_lst.append(np.median(allocation_history['min_cost']))
            average_cost = np.average(cost_lst)
            exp_data[idx,:] = [k, average_cost]
            idx +=1
        k = exp_data[:,0]
        average_cost = exp_data[:,1]
        fig = plt.figure()
        ax = fig.add_subplot('111')
        fig.suptitle(fig_title)
        ax.scatter(k,average_cost)
        # ax.errorbar(k,average_cost, average_std,capsize = 3, fmt="" ,ecolor='k')
        ax.set_title('Cost as Function of K')
        ax.set_xlabel('K')
        ax.set_ylabel('Cost')
        ax.grid(axis='y')
        plt.show()   


    if cost_as_threshold:
        exp_data = np.zeros([len(threshold_lst),2], dtype=float)
        idx = 0
        for thersh in threshold_lst:
            average_cost_lst = []
            for k in k_lst:
                data = np.load(url + 'ros_sim_data_k_'+str(k)+'_thresh_'+str(thersh)+".npy", allow_pickle=True)
                data = data.item()
                general_data, drone_data = data['general_data'],  data['drone_data']
                allocation_history = general_data['allocation_history']
                average_cost_lst.append(np.median(allocation_history['min_cost']))
            average_cost = np.average(average_cost_lst)
            exp_data[idx,:] = [thersh, average_cost]
            idx +=1
        thresh = exp_data[:,0]
        average_cost = exp_data[:,1]
        fig = plt.figure()
        ax = fig.add_subplot('111')
        ax.scatter(thresh, average_cost)
        ax.set_xlabel('Treshold')
        ax.set_ylabel('Cost')
        ax.set_title(f'{fig_title} \n Cost as Function of Threshold')
        ax.grid(axis='y')
        plt.show()   



if relative_distance:
    dot_size = 20
    dot_color = 'b'
    line_color = 'g'

    fig = plt.figure()
    ax = fig.add_subplot('111')
    x = [0.5, 1, 1.5,0.5]
    y = [0.5, 1, 0.5,0.5]
    ax.scatter(x,y,s=dot_size,c=dot_color)
    ax.plot(x,y,'--',c=line_color)

    fig1 = plt.figure()
    ax1 = fig1.add_subplot('111')
    x = [0.5,0.7,1.2,1.3,0.5]
    y= [0.5,1.5,1.2,0.6,0.5]
    ax1.plot(x,y,'--',c=line_color)
    ax1.scatter(x,y,s=dot_size,c=dot_color)
    x = [0.5,1.2]
    y= [0.5,1.2]
    ax1.plot(x,y,'--',c=line_color)
    ax1.scatter(x,y,s=dot_size,c=dot_color)
    x = [0.7,1.3]
    y= [1.5,0.6]
    ax1.plot(x,y,'--',c=line_color)
    ax1.scatter(x,y,s=dot_size,c=dot_color)
    ax1.set_title('Relative Distance - 4 Targets')
    ax1.set_xlabel('X(m)')
    ax1.set_ylabel('Y(m)')
    plt.show()