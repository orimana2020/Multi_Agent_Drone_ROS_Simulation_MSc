import numpy as np
import matplotlib.pyplot as plt
from params import colors


task_num = 7

# ------------- LOAD DATA -----------------
# Allocatio
general_data, drone_data = np.load("task_"+str(task_num)+"_data.npy", allow_pickle=True)
initial_drone_num, task_time, allocation_data = general_data
min_dist_lst = []
threshold_lst = []
combination_lst =[] 
drone_num_lst =[]
for row in allocation_data:
    min_dist, threshold, combination ,drone_num = row
    min_dist_lst.append(min_dist)
    threshold_lst.append(threshold)
    combination_lst.append(combination) 
    drone_num_lst.append(drone_num)

min_dist = np.array(min_dist_lst)
threshold = np.array(threshold_lst)
combination = np.array(combination_lst)
drone_num = np.array(drone_num_lst)
idx = np.array(range(0,len(min_dist)))
#check drone changed idx
current_drone_num = initial_drone_num
drone_change_idx  = []
for i, drone in enumerate(drone_num):
    if drone != current_drone_num:
        drone_change_idx.append(i)
        current_drone_num = drone
drone_change_idx = [x-0.5 for x in drone_change_idx]


# timming 



# ------------- plotting ------------------
# Allocation
fig1 = plt.figure()
ax1 = fig1.add_subplot('111')
ax1.scatter(idx, min_dist,c='blue', label='min_dist')
ax1.scatter(idx, threshold, c='red', label='Threshold')
ax1.vlines(x=drone_change_idx, ymin=0, ymax=max(min_dist), colors='purple', ls='--', lw=2, label='Drone Num changed')
ax1.set_xlabel("Iteration")
ax1.set_ylabel("Distance [m]")
ax1.set_title('Allocation Performance')
ax1.legend()


# timming

fig2, ax2 = plt.subplots()
offset = np.array([1,2,3,4])
j=0
for drone_id, drone in enumerate(drone_data): 
    # to_target, at_target, to_base, at_base, target_idx, targets_num = drone
    ax2.bar(np.array([1,2,3,4]) + j, drone[:4] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[drone_id], label='Drone_'+str(drone_id))
    j += 0.2
ax2.set_xticks(np.array([1,2,3,4,5]))   
ax2.set_xticklabels(['to_target', 'at_target', 'to_base', 'at_base', 'targets_num'])

ax2.set_ylabel("Time [sec]")
ax2.set_title('Drones performance')
ax2.legend()

ax3 = ax2.twinx()
j=0
for drone_id, drone in enumerate(drone_data): 
    # to_target, at_target, to_base, at_base, target_idx, targets_num = drone
    ax3.bar(5 + j, drone[-1] , width=0.1, edgecolor="white", linewidth=0.1, color=colors[drone_id], label='Drone_'+str(drone_id))
    j += 0.2
ax3.set_ylabel('Targets Num')
plt.show()


