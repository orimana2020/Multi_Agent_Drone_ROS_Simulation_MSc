#! /usr/bin/env python3

# ------------- version ----------------
# full simulation works for ros noetic on ubuntu 20.04
# python 3.8.10


# -------- how to run------------>
# terminal 1 : $ roslaunch rotors_gazebo drone_poll_lanch.launch  
# terminal 2: $ rosrun multi_agent_task_allocation main_ros_sim.py 

import rospy
from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Allocation_algorithm_init import  Env, Targets ,get_figure
import numpy as np
from drone_flight_manager import Flight_manager
plt.ion()

def main():
    targets = Targets(targets_num=70,data_source='circle')   # !!! need update - this data should come from camera
    z_span, y_span, x_span = targets.span 
    drone_num = 3
    safety_distance_trajectory = 0.4
    safety_distance_allocation = safety_distance_trajectory * 2
    ta = Allocation(drone_num, targets, safety_distance_allocation , k_init=5, magazine=[10,10,10]) 
    path_planner = Trajectory(x_span, y_span ,z_span ,drone_num=ta.drone.drone_num, res=0.1, safety_distance=safety_distance_trajectory)
    fig = get_figure(targets, ta.drone)
    fc = Flight_manager(lin_velocity=2.5, drone_num=drone_num)
    rate = rospy.Rate(10)

    for drone_idx in range(drone_num):
        fc.take_off(drone_idx=drone_idx, height=0.5)
    rospy.sleep(5)

    start = []
    goal = []

    for _ in range(ta.drone.drone_num):
        start.append([])
        goal.append([])
    current_pos = ta.drone.base[:]
    current_pos_title = ta.drone.start_title[:]

    is_reached_goal = np.zeros(ta.drone.drone_num, dtype=int)
    path_found = np.zeros(ta.drone.drone_num, dtype=int)
    allocation = None
    allocation_availability = np.zeros(ta.drone.drone_num, dtype=int)
    at_base = np.zeros(ta.drone.drone_num, dtype=int)

    while ta.optim.unvisited_num > 0:
        print('unvisited = %d' %ta.optim.unvisited_num)
        # ------------------------     update magazine state & allocate new targets -------- #    
        for j in range(ta.drone.drone_num):
            if allocation_availability[j] == 1:
                change_flag = np.zeros(ta.drone.drone_num, dtype=int)
                change_flag[j] = 1
                allocation = ta.allocate(change_flag)
                if allocation == 'update_kmeans':
                    break
        
        if allocation == 'update_kmeans':
            for j in range(ta.drone.drone_num):
                if not(at_base[j] == 1):
                    if (path_found[j] == 0) and (ta.optim.unvisited[ta.optim.current_targets[j]] == False):
                        ta.drone.start_title[j] = current_pos_title[j]
                        start[j] = current_pos[j]
                        ta.drone.goal_title[j] = 'base'
                        goal[j] = ta.drone.base[j]
                    
                    if (path_found[j] == 0) and (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
                        ta.drone.start_title[j] = current_pos_title[j]
                        start[j] = current_pos[j]
                        ta.drone.goal_title[j] = 'target'
                        goal[j] = tuple(targets.targetpos[ta.optim.current_targets[j],:])


                
        # --------------------------- KMEANS ------------------------ #            
        while allocation == 'update_kmeans':
            print('-------kmeans mode-------')
            for j in range(ta.drone.drone_num):
                is_reached_goal[j] = fc.reached_goal(drone_idx=j) 
                if not (at_base[j] == 1):
                    if  (ta.drone.goal_title[j] == 'target') and (path_found[j] == 1) and (is_reached_goal[j] == 1) and (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
                        path_found[j] = 0
                        ta.optim.unvisited_num -= 1
                        ta.optim.unvisited[ta.optim.current_targets[j]] = False
                        ta.optim.update_history(ta.optim.current_targets[j], j, ta.targets) 
                        ta.targets.targetpos_reallocate[ta.optim.current_targets[j], :] = np.inf
                        ta.optim.update_distance_mat(ta.optim.current_targets[j])
                        ta.drone.start_title[j] = 'target'
                        ta.drone.goal_title[j] = 'base' 
                        start[j] = tuple(targets.targetpos[ta.optim.current_targets[j],:])
                        goal[j] = ta.drone.base[j]
                        is_reached_goal[j] = 0 # just to reset 

                    # find path to target
                    if (path_found[j] == 0) and (ta.drone.goal_title[j] == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True) :
                        path_found[j] = path_planner.plan(start[j], goal[j] ,ta.drone.start_title[j], ta.drone.goal_title[j] ,drone_idx=j, drone_num=ta.drone.drone_num, at_base=at_base)
                        if path_found[j] == 1:
                            fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)
                    # find path to base
                    if (path_found[j] == 0) and  (ta.optim.unvisited[ta.optim.current_targets[j]] == False) :
                        path_found[j] = path_planner.plan(start[j], goal[j] ,ta.drone.start_title[j], ta.drone.goal_title[j] ,drone_idx=j, drone_num=ta.drone.drone_num, at_base=at_base)
                        if path_found[j] == 1:
                            fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)
                        if path_found[j] == 0:
                            fig.plot_no_path_found(start[j], goal[j])  

                    if ((path_found[j] == 1) and (ta.drone.goal_title[j] == 'base') and (is_reached_goal[j] == 1)):
                        at_base[j] = 1
            fig.ax.axes.clear()
            fig.plot_at_base(drone_num, at_base)
            fig.plot_all_targets()
            fig.plot_trajectory(path_planner,path_found, ta.drone.drone_num,goal_title=ta.drone.goal_title, path_scatter=0, smooth_path_cont=1, smooth_path_scatter=0, block_volume=1)
            fig.plot_history(ta.optim.history, drone_num, ta.drone.colors)
            fig.show(sleep_time=0.7)
            if np.sum(at_base[:ta.drone.drone_num]) == ta.drone.drone_num :
                for j in range(ta.drone.drone_num):
                    ta.drone.current_magazine[j] = ta.drone.full_magazine[j]
                    ta.drone.start_title[j] = 'base'
                    ta.drone.goal_title[j] = 'target'
                    current_pos_title[j] = 'base'
                    current_pos[j] = ta.drone.base[j]
                    is_reached_goal[j] = 0
                    path_found[j] = 0 
                    allocation_availability[j] = 0
                ta.update_kmeans()
                allocation = None  
            rate.sleep()

        #  --------------------------------    path planning ----------------------------- #
        fig.ax.axes.clear()
        for j in range(ta.drone.drone_num):
            if (path_found[j] == 0) and (ta.optim.unvisited_num > 0): #force trying plan to base until is found
                ta.drone.start_title[j] = current_pos_title[j]
                start[j] = current_pos[j]
                if ta.drone.current_magazine[j] > 0:
                    ta.drone.goal_title[j] = 'target'
                    goal[j] = tuple(targets.targetpos[ta.optim.current_targets[j], :])
                else:
                    ta.drone.goal_title[j] = 'base'
                    goal[j] = ta.drone.base[j]
                    
                path_found[j] = path_planner.plan(start[j], goal[j] ,ta.drone.start_title[j], ta.drone.goal_title[j] ,drone_idx=j, drone_num=ta.drone.drone_num, at_base=at_base)
                if path_found[j] == 1:
                    at_base[j] = 0
                    current_pos_title[j] = None
                    fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)

                if path_found[j] == 0:
                    fig.plot_no_path_found(start[j], goal[j])  
                    
            is_reached_goal[j] = fc.reached_goal(drone_idx=j) 
            fig.plot_trajectory(path_planner,path_found, ta.drone.drone_num,goal_title=ta.drone.goal_title, path_scatter=0, smooth_path_cont=1, smooth_path_scatter=0, block_volume=1)


            if (is_reached_goal[j] == 1) and (path_found[j] == 1):
                path_found[j] = 0
                # arrived base
                if ta.drone.goal_title[j] == 'base':
                    current_pos_title[j] = 'base'
                    current_pos[j] = ta.drone.base[j]
                    ta.drone.current_magazine[j] = ta.drone.full_magazine[j]
                    at_base[j] = 1
                    allocation_availability[j] = 1

                # arrived target
                elif ta.drone.goal_title[j] == 'target':
                    current_pos_title[j] = 'target'
                    current_pos[j] = tuple(targets.targetpos[ta.optim.current_targets[j],:])
                    ta.drone.current_magazine[j] -= 1
                    ta.optim.unvisited_num -= 1
                    ta.optim.unvisited[ta.optim.current_targets[j]] = False
                    ta.optim.update_history(ta.optim.current_targets[j], j, ta.targets) 
                    ta.targets.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])
                    if ta.drone.current_magazine[j] > 0:
                        allocation_availability[j] = 1
                    else:
                        allocation_availability[j] = 0         
            else:
                allocation_availability[j] = 0
                    
        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
        fig.plot_at_base(drone_num, at_base)
        fig.plot_all_targets()
        fig.plot_history(ta.optim.history, drone_num, ta.drone.colors)
        fig.show(sleep_time=0)
        rate.sleep()


    # -------------------------------- Return all drones to base

    while not (np.sum(at_base) == drone_num):
        print('return all drones to base')
        for j in range(ta.drone.drone_num):
            is_reached_goal[j] = fc.reached_goal(drone_idx=j) 

            if not (at_base[j] == 1):
                if (current_pos_title[j] == 'target') and (path_found[j] == 0):
                    ta.drone.start_title[j] = current_pos_title[j]
                    start[j] = current_pos[j]
                    ta.drone.goal_title[j] = 'base'
                    goal[j] = ta.drone.base[j]
                    path_found[j] = path_planner.plan(start[j], goal[j] ,ta.drone.start_title[j], ta.drone.goal_title[j] ,drone_idx=j, drone_num=ta.drone.drone_num, at_base=at_base)
                    if path_found[j] == 1:
                        fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)


                if (is_reached_goal[j] == 1) and (path_found[j] == 1):
                    at_base[j] = 1
                    ta.targets.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])
        fig.ax.axes.clear()
        fig.plot_at_base(drone_num, at_base)
        fig.plot_history(ta.optim.history, drone_num, ta.drone.colors)
        fig.plot_all_targets()    
        fig.plot_trajectory(path_planner,path_found, ta.drone.drone_num,goal_title=ta.drone.goal_title, path_scatter=0, smooth_path_cont=1, smooth_path_scatter=0, block_volume=1)
        fig.show(sleep_time=1)
    fig.plot_at_base(drone_num, at_base)
    fig.plot_all_targets()
    print('!! finnitto !!!')
    fig.plot_history(ta.optim.history, drone_num, ta.drone.colors)
    fig.show(sleep_time=13)


if __name__ == '__main__':
    rospy.init_node('send_my_command', anonymous=True)
    rospy.sleep(3)
    main()
    


