#! /usr/bin/env python3

# ------------- version ----------------
# python 3.8.10

from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Allocation_algorithm_init import  Env, Targets ,get_figure
import numpy as np
import time
plt.ion()


ploting  = False

def main():
    drone_num = 3
    drone_performace = np.array([60,100,70])
    safety_distance_allocation = 0
    k = 3
    targets = Targets(targets_num=150,data_source='dataset')   # !!! need update - this data should come from camera
    z_span, y_span, x_span = targets.span 
    ta = Allocation(drone_num, targets, safety_distance_allocation , k_init=k, magazine=[10,10,10]) 
    fc = Env(drone_performace=drone_performace, drone_num=ta.drone.drone_num) # !!! need update, data come from ros real env
    if ploting:
        fig = get_figure(targets, ta.drone)
    is_reached_goal = np.zeros(ta.drone.drone_num, dtype=int)
    allocation = None

    while ta.optim.unvisited_num > 0:
        print('unvisited = %d' %ta.optim.unvisited_num)
        change_flag = np.zeros(ta.drone.drone_num, dtype=int)
        for j in range(ta.drone.drone_num):
            # not valid at first itr
            if is_reached_goal[j] == 1:
                change_flag[j] = 1
        allocation = ta.allocate(change_flag)

                
        # --------------------------- KMEANS ------------------------ #            
        while allocation == 'update_kmeans':
            print('-------kmeans mode-------')
            for j in range(ta.drone.drone_num):
                is_reached_goal[j] = fc.reached_goal(drone_idx=j) 
                if  (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
                    ta.optim.unvisited_num -= 1
                    ta.optim.unvisited[ta.optim.current_targets[j]] = False
                    ta.optim.update_history(ta.optim.current_targets[j], j, ta.targets) 
                    ta.targets.targetpos_reallocate[ta.optim.current_targets[j], :] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])

            ta.update_kmeans()
            allocation = None  
            if ploting:
                fig.ax.axes.clear()
                fig.plot_all_targets()
                fig.plot_history(ta.optim.history, drone_num, ta.drone.colors)
                fig.show(sleep_time=0.1)

    
        #  --------------------------------    update ----------------------------- #
        if ploting:
            fig.ax.axes.clear()
        for j in range(ta.drone.drone_num):
            is_reached_goal[j] = fc.reached_goal(drone_idx=j) 
            if (is_reached_goal[j] == 1) :
                ta.optim.unvisited_num -= 1
                ta.optim.unvisited[ta.optim.current_targets[j]] = False
                ta.optim.update_history(ta.optim.current_targets[j], j, ta.targets) 
                ta.targets.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                ta.optim.update_distance_mat(ta.optim.current_targets[j])
        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
        if ploting:
            fig.plot_all_targets()
            fig.plot_history(ta.optim.history, drone_num, ta.drone.colors)
            fig.show(sleep_time=0)

    print('finished')  


    # analysis
    plt.ioff()
    fig1, ax1 = plt.subplots()    
    x = np.array([range(len(ta.optim.min_dist_step))])
    y = np.array(ta.optim.min_dist_step)
    print('k = ', k)
    print('average distance allocation = ', np.sum(y)/len(ta.optim.min_dist_step))
    print('min distance allocation = ', min(y))
    ax1.axes.set_ylim(bottom=0, top=max(y)+0.5) 
    ax1.scatter(x,y)
    ax1.set_title('minimum allocation distance')
    ax1.set_xlabel('instance')
    ax1.set_ylabel('minimum allocation distance')

    fig2, ax2 = plt.subplots() 
    ax2.set_title('Histogram')
    ax2.set_ylabel('instances')
    ax2.set_xlabel('minimum allocation distance')
    ax2.hist(y)
    fig2.show()
    plt.show()
       


if __name__ == '__main__':
    main()
    


