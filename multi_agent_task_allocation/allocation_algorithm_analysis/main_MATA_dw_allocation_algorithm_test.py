#! /usr/bin/env python3

# ------------------------ how to run  simulation-----------------------
# terminal 1 : $ roslaunch rotors_gazebo experiment1.launch 
# terminal 2: $ rosrun multi_agent_task_allocation main_MATA_dw.py
# -------------------------------------------------------------



from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure , Drone_Manager, Logger, Analysis
import numpy as np
import params

def main(i,k,threshold):
    logger = Logger()
    an = Analysis()
    ta = Allocation(logger, an, k ,threshold) # compute fisrt targets allocation in init
    dm = Drone_Manager(params.uri_list, params.base, params.magazine, ta)
    ta.update_kmeans(dm);  dm.update_first_goals(ta);
    an.start(dm)
    allocation = 'allocate'

    while ta.optim.unvisited_num > 0:
        # ------------------------Target Allocation-------------------------- #   
        if allocation == 'allocate':
            for j in range(ta.drone_num):
                if True:
                    change_flag = np.zeros(ta.drone_num, dtype=int)
                    change_flag[j] = 1
                    allocation = ta.allocate(change_flag)
                    if allocation == 'update_kmeans':
                        break
                        
                    elif allocation == 'remove_drone':
                        break
            
        if allocation == 'update_kmeans':
            ta.update_kmeans(dm)
            allocation = 'allocate'

        if allocation == 'remove_drone':
            ta.drone_num -= 1
            ta.drone_num_changed = True
            ta.update_kmeans(dm)
            allocation = 'update_kmeans'
        
        for j in range(ta.drone_num):
            ta.optim.unvisited_num -= 1
            ta.optim.unvisited[ta.optim.current_targets[j]] = False
            ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
            ta.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
            ta.optim.update_distance_mat(ta.optim.current_targets[j])

        print(ta.optim.current_targets)

    an.save(i,k,threshold)

    

if __name__ == '__main__':
    k_lst = [2,3,4,5,6,7,8]
    threshold_lst = [0.5,0.6,0.7,0.8,0.9]
    for k in k_lst:
        for thresh in threshold_lst:
            for i in range(5):
                main(i,k,thresh)
