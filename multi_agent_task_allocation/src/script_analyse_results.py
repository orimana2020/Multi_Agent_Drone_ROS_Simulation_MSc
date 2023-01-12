import numpy as np
import matplotlib.pyplot as plt


colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']

# ------------- LOAD DATA -----------------
k_init = 2
threshold_factor = 0.9
data = np.load('task_k_'+str(k_init)+'_threshold_'+str(threshold_factor)+'_1'+"_data.npy", allow_pickle=True)
data = data.item()
general_data, drone_data = data['general_data'],  data['drone_data']

# Allocatio
allocation_history = general_data['allocation_history']
min_dist = allocation_history['min_dist']
threshold = allocation_history['threshold']
combination = allocation_history['combination']
drone_num = allocation_history['drone_num']
kmeans = allocation_history['is_kmeans']
safety_distance_allocation = general_data['safety_distance_allocation']
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

# # ------------- plotting ------------------
# Allocation
fig1 = plt.figure()
fig1.suptitle(f'k: {k_init}, threshold factor: {threshold_factor}, task_time: {round(general_data["total_task_time"], 2)} [sec]')
ax1 = fig1.add_subplot('111')
ax1.scatter(idx, min_dist,c='blue', label='min_dist',s=2)
ax1.scatter(idx, threshold, c='green', label='Threshold',s=2)
ax1.vlines(x=drone_change_idx, ymin=0, ymax=max(min_dist), colors='purple', ls='--', lw=1, label='Drone Num changed')
ax1.vlines(x=kmeans_idx, ymin=0, ymax=max(min_dist), colors='yellow', ls='--', lw=0.5, label='KMEANS')
ax1.hlines(y=safety_distance_allocation, xmin=0, xmax=max(idx), colors='red',ls='--', lw=2, label='safety distance')
ax1.set_xlabel("Iteration")
ax1.set_ylabel("Distance [m]")
ax1.set_title('Allocation Performance')
ax1.legend()


fig2, ax2 = plt.subplots()
i=j=0
for drone in drone_data.keys(): 
    ax2.bar(1 + j, drone_data[drone]['time_to_target'] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[i], label='Drone_'+str(i))
    ax2.bar(2 + j, drone_data[drone]['time_at_target'] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[i])
    ax2.bar(3 + j, drone_data[drone]['time_to_base'] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[i])
    ax2.bar(4 + j, drone_data[drone]['time_at_base'] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[i])
    j += 0.2
    i += 1

ax2.set_xticks(np.array([1,2,3,4,5]))   
ax2.set_xticklabels(['to_target', 'at_target', 'to_base', 'at_base', 'targets_num'])

ax2.set_ylabel("Time [sec]")
ax2.set_title('Drones performance')
ax2.legend()

ax3 = ax2.twinx()
j=i=0
for drone_id, drone in enumerate(drone_data): 
    ax3.bar(5 + j, drone_data[drone]['visited_targets_num'] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[i])
    j += 0.2
    i += 1
ax3.set_ylabel('Targets Num')
plt.show()


