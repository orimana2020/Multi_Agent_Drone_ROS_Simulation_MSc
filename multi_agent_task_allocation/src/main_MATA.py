#! /usr/bin/env python3


# -------- how to run sim mode ------------>
# full simulation works for ros noetic on ubuntu 20.04
# python 3.8.10
# terminal 1 : $ roslaunch rotors_gazebo drone_poll_lanch.launch  
# terminal 1 : $ roslaunch rotors_gazebo drone_poll_circle.launch 

# terminal 2: $ rosrun multi_agent_task_allocation main_MATA.py
# ----------------------------------------->

from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure ,Get_Drones
import numpy as np
import params
plt.ion()

if params.mode == 'sim':
    from rotors_flight_manager import Flight_manager
    import rospy
    rospy.init_node('send_my_command', anonymous=True)
    rospy.sleep(3)
elif params.mode == 'cf':
    from CF_Flight_Manager import Flight_manager

def main():
    ta = Allocation() 
    fig = get_figure()
    drones = Get_Drones(params.uri_list, params.base, params.magazine, ta.drone_num)
    fc = Flight_manager(ta.drone_num)
    path_planner = Trajectory(drones)
    fc.take_off_swarm()
    allocation = None

    while ta.optim.unvisited_num > 0:
        print(f'unvisited targets = {ta.optim.unvisited_num}')
        # ------------------------     update magazine state & allocate new targets -------- #    
        for j in range(ta.drone_num):
            if drones[j].is_available:
                change_flag = np.zeros(ta.drone_num, dtype=int)
                change_flag[j] = 1
                allocation = ta.allocate(change_flag)
                if allocation == 'update_kmeans':
                    print('kmeans mode started')
                    break
        
        if allocation == 'update_kmeans':
            for j in range(ta.drone_num):
                if not(drones[j].at_base):
                    if (not drones[j].path_found) and (ta.optim.unvisited[ta.optim.current_targets[j]] == False):
                        drones[j].start_title = drones[j].current_pos_title
                        drones[j].start_coords = drones[j].current_pos_coords
                        drones[j].goal_title = 'base'
                        drones[j].goal_coords = drones[j].base
                        
                    if (not drones[j].path_found) and (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
                        drones[j].start_title = drones[j].current_pos_title
                        drones[j].start = drones[j].current_pos_coords
                        drones[j].goal_title = 'target'
                        drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])

        # --------------------------- KMEANS ------------------------ #      
        while allocation == 'update_kmeans':
            for j in range(ta.drone_num):
                print(f'checkpoint  0 drone {j}')
                drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords) 
                if not (drones[j].at_base):
                    print(f'drone {j} not at_base')
                    # arrived to target
                    if  (drones[j].goal_title == 'target') and (drones[j].path_found) and (drones[j].is_reached_goal) and (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
                        print(f'drone {j} arrived to target')
                        drones[j].path_found = 0
                        ta.optim.unvisited_num -= 1
                        ta.optim.unvisited[ta.optim.current_targets[j]] = False
                        ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
                        ta.targetpos_reallocate[ta.optim.current_targets[j], :] = np.inf
                        ta.optim.update_distance_mat(ta.optim.current_targets[j])
                        drones[j].start_title = 'target'
                        drones[j].goal_title = 'base' 
                        drones[j].start_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                        drones[j].goal_coords = drones[j].base
                        drones[j].is_reached_goal = 0 

                    print('checkpoint 1')
                    # find path to target
                    if not (drones[j].path_found) and (drones[j].goal_title == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True) and (not (fc.open_threads[j].is_alive())):
                        print(f'drone {j} try find path to target')
                        drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                        if drones[j].path_found:
                            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                    
                    print('checkpoint 2')
                    # find path to base
                    if not (drones[j].path_found) and  (ta.optim.unvisited[ta.optim.current_targets[j]] == False) and (not (fc.open_threads[j].is_alive())) :
                        print(f'drone {j} try find path to base')
                        drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                        if drones[j].path_found:
                            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                        if not drones[j].path_found:
                            fig.plot_no_path_found(drones[j])  
                    
                    print('checkpoint 3')
                    if ((drones[j].path_found) and (drones[j].goal_title == 'base') and (drones[j].is_reached_goal)):
                        drones[j].at_base = 1
            print('checkpoint 4')            
            fig.ax.axes.clear()
            fig.plot_all_targets()
            fig.plot_trajectory(path_planner, drones ,ta.drone_num)
            fig.plot_history(ta.optim.history)
            fig.show()

            k_means_permit = True
            for j in range(ta.drone_num):
                if not drones[j].at_base:
                    k_means_permit = False
            print(f'kmeans permit {k_means_permit}')
            if k_means_permit :
                for j in range(ta.drone_num):
                    print(f'drone {j} at_base: {drones[j].at_base}')
                    drones[j].current_magazine = drones[j].full_magazine
                    drones[j].start_title = 'base'
                    drones[j].goal_title = 'target'
                    drones[j].current_pos_title = 'base'
                    drones[j].current_pos_coords = drones[j].base
                    drones[j].is_reached_goal = 0
                    drones[j].path_found = 0 
                    drones[j].is_available = 0
                current_drone_num = ta.drone_num
                ta.update_kmeans()
                print('kmeans updated')
                allocation = None 
                if current_drone_num > ta.drone_num: #land inactive drones
                    for j in range(current_drone_num-1, ta.drone_num-1,-1):
                        fc.land(drone_idx=j)
                        drones[j].is_active = False
                        print(f'drone {j} is landing')
            fc.sleep()
           
        #  --------------------------------    path planning ----------------------------- #
        fig.ax.axes.clear()
        for j in range(ta.drone_num):
            if not (drones[j].path_found) and (ta.optim.unvisited_num > 0) and (not (fc.open_threads[j].is_alive())): #force trying plan to base until is found
                drones[j].start_title = drones[j].current_pos_title
                drones[j].start_coords = drones[j].current_pos_coords
                if drones[j].current_magazine > 0:
                    drones[j].goal_title = 'target'
                    drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j], :])
                else:
                    drones[j].goal_title = 'base'
                    drones[j].goal_coords = drones[j].base
                    
                drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                if drones[j].path_found:
                    drones[j].at_base = 0
                    drones[j].current_pos_title = None
                    fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])

                if not drones[j].path_found:
                    fig.plot_no_path_found(drones[j])  
                    
        
            drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords) 
            if (drones[j].is_reached_goal) and (drones[j].path_found):
                drones[j].path_found = 0
                # arrived base
                if drones[j].goal_title == 'base':
                    drones[j].current_pos_title = 'base'
                    drones[j].current_pos_coords = drones[j].base
                    drones[j].current_magazine = drones[j].full_magazine
                    drones[j].at_base = 1
                    drones[j].is_available = 1

                # arrived target
                elif drones[j].goal_title == 'target':
                    drones[j].current_pos_title = 'target'
                    drones[j].current_pos_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                    drones[j].current_magazine -= 1
                    ta.optim.unvisited_num -= 1
                    ta.optim.unvisited[ta.optim.current_targets[j]] = False
                    ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
                    ta.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])
                    if drones[j].current_magazine > 0:
                        drones[j].is_available = 1
                    else:
                        drones[j].is_available = 0         
            else:
                drones[j].is_available = 0

            fig.plot_trajectory(path_planner, drones ,ta.drone_num)  
        fig.plot_all_targets()
        fig.plot_history(ta.optim.history)
        fig.show()
        fc.sleep()
        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
    # -------------------------------- Return all drones to base
    all_at_base = True
    for j in range(ta.drone_num):
        if not drones[j].at_base:
            all_at_base = False
    while not all_at_base:
        print('return all drones to base')
        for j in range(ta.drone_num):
            drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords) 

            if not (drones[j].at_base):
                if (drones[j].current_pos_title == 'target') and not (drones[j].path_found) and (not (fc.open_threads[j].is_alive())):
                    drones[j].start_title = drones[j].current_pos_title
                    drones[j].start_coords = drones[j].current_pos_coords
                    drones[j].goal_title = 'base'
                    drones[j].goal_coords = drones[j].base
                    drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                    if drones[j].path_found:
                        fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                        
                drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords)
                if (drones[j].is_reached_goal) and (drones[j].path_found):
                    drones[j].at_base = 1
                    ta.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])
        all_at_base = True
        for j in range(ta.drone_num):
            if not drones[j].at_base:
                all_at_base = False
        fig.ax.axes.clear()
        fig.plot_history(ta.optim.history)
        fig.plot_all_targets()    
        fig.plot_trajectory(path_planner, drones ,ta.drone_num)
        fig.show()
        fc.sleep()
    fc.land('all', drones)
    fig.plot_all_targets()
    fig.plot_history(ta.optim.history)
    fig.show()
    print(' Task Done Successfully')


if __name__ == '__main__':
    main()
    


