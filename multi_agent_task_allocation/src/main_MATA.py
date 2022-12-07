#! /usr/bin/env python3
# full simulation works for ros noetic on ubuntu 20.04 , python 3.8.10
# -------- how to run sim mode ------------>
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
    ta = Allocation() # compute fisrt targets allocation
    fig = get_figure()
    drones = Get_Drones(params.uri_list, params.base, params.magazine, ta)
    fc = Flight_manager(ta.drone_num)
    path_planner = Trajectory(drones)
    fc.take_off_swarm()
    allocation = None

    while ta.optim.unvisited_num > 0:
        
        # ------------------------Allocate targets -------- #   
        print(f'unvisited targets = {ta.optim.unvisited_num}') 
        for j in range(ta.drone_num):
            if drones[j].is_available:
                change_flag = np.zeros(ta.drone_num, dtype=int)
                change_flag[j] = 1
                allocation = ta.allocate(change_flag)
                if allocation == 'update_kmeans':
                    print(f'kmeans mode started, j = {j}')
                    break
                else:
                    drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                    

        # --------------------------- UPDATE KMEANS ------------------------- #  
        while allocation == 'update_kmeans':
            k_means_permit = False
            while not k_means_permit:
                k_means_permit = True
                for j in range(ta.drone_num):
                    if (not drones[j].at_base ) or (ta.optim.unvisited[ta.optim.current_targets[j]] == True) :
                        k_means_permit = False
                print(f'kmeans permit {k_means_permit}')
                for j in range(ta.drone_num):
                    if (drones[j].goal_coords == None) and (drones[j].start_title == 'base'):
                        continue
                    elif (drones[j].goal_coords == None) and drones[j].start_title == 'target':
                        drones[j].goal_coords = drones[j].base
                        drones[j].goal_title = 'base'
                        drones[j].is_reached_goal = 0
                    else:
                        drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal=drones[j].goal_coords, title=drones[j].goal_title) 
                    # print(f'drone {j}, start: {drones[j].start_title}, goal: {drones[j].goal_title}, reach_goal = {drones[j].is_reached_goal} , path found: {drones[j].path_found}, target_unvisited: {ta.optim.unvisited[ta.optim.current_targets[j]]} ' )
                    
                    # find path to unvisited target
                    if (not (drones[j].path_found)) and (drones[j].goal_title == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True) and (not (fc.open_threads[j].is_alive())):
                        drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                        if drones[j].path_found:
                            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])

                    # arrived to target
                    elif  (drones[j].goal_title == 'target') and (drones[j].is_reached_goal) :
                        ta.optim.unvisited_num -= 1
                        ta.optim.unvisited[ta.optim.current_targets[j]] = False
                        ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
                        ta.targetpos_reallocate[ta.optim.current_targets[j], :] = np.inf
                        ta.optim.update_distance_mat(ta.optim.current_targets[j])
                        drones[j].path_found = 0
                        drones[j].start_title = 'target' 
                        drones[j].start_coords = tuple(fc.get_position(j))
                        drones[j].is_reached_goal = 0 
                        drones[j].current_magazine -= 1
                        drones[j].goal_title = 'base'
                        drones[j].goal_coords = drones[j].base
                        drones[j].at_base = 0
                        
                    # find path to base / intermidiate
                    elif (not (drones[j].path_found)) and drones[j].goal_title == 'base'  and (not (fc.open_threads[j].is_alive())) :
                        drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                        if drones[j].path_found:
                            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])

                    # arrived to base / intermidiate
                    elif ((drones[j].goal_title == 'base') and (drones[j].is_reached_goal)):
                        drones[j].at_base = 1
                        drones[j].goal_coords = None
                        drones[j].start_title = 'base'

                    fig.ax.axes.clear()
                    fig.plot_all_targets()
                    fig.plot_trajectory(path_planner, drones ,ta.drone_num)
                    fig.plot_history(ta.optim.history)
                    fig.show()
                    fc.sleep()

            if k_means_permit :
                for j in range(ta.drone_num):
                    drones[j].start_title = 'base'
                    drones[j].start_coords = tuple(fc.get_position(j))
                    drones[j].current_magazine = drones[j].full_magazine
                    drones[j].goal_title = 'target'
                    drones[j].is_reached_goal = 0
                    drones[j].path_found = 0 
                    drones[j].is_available = 0
                current_drone_num = ta.drone_num
                ta.update_kmeans()
                for j in range(ta.drone_num):
                    drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                print('kmeans updated')
                allocation = None 
                if current_drone_num > ta.drone_num: #land inactive drones
                    for j in range(current_drone_num-1, ta.drone_num-1,-1):
                        fc.land(drone_idx=j)
                        drones[j].is_active = False
                        print(f'drone {j} is landing')
            
           
        #  -------------------------------- PATH PLANNING ------------------------------------------ #
        fig.ax.axes.clear()
        for j in range(ta.drone_num):
            if not (drones[j].path_found) and (ta.optim.unvisited_num > 0) and (not (fc.open_threads[j].is_alive())):
                drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                if drones[j].path_found:
                    drones[j].at_base = 0
                    fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                else:
                    fig.plot_no_path_found(drones[j])  
                    
        # ------------------ UPDATE DRONES STATUS -------------------------------------------#
            drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords, title=drones[j].goal_title) 
            if (drones[j].is_reached_goal) and (drones[j].path_found):
                
                # arrived base
                if drones[j].goal_title == 'base':
                    drones[j].start_title = 'base'
                    drones[j].start_coords = tuple(fc.get_position(j))
                    drones[j].goal_title = 'target'
                    drones[j].goal_coords = None
                    drones[j].current_magazine = drones[j].full_magazine
                    drones[j].at_base = 1
                    drones[j].is_available = 1
                    drones[j].path_found = 0
                    drones[j].is_reached_goal = 0
              
                # arrived target
                elif drones[j].goal_title == 'target':
                    drones[j].start_title = 'target'
                    drones[j].start_coords = tuple(fc.get_position(j))
                    drones[j].current_magazine -= 1
                    drones[j].path_found = 0
                    drones[j].is_reached_goal = 0
                    ta.optim.unvisited_num -= 1
                    ta.optim.unvisited[ta.optim.current_targets[j]] = False
                    ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
                    ta.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])
                    if drones[j].current_magazine > 0:
                        drones[j].is_available = 1
                        drones[j].goal_title = 'target'
                        drones[j].goal_coords = None
                    else:
                        drones[j].is_available = 0   
                        drones[j].goal_title = 'base'
                        drones[j].goal_coords = drones[j].base      
            else:
                drones[j].is_available = 0

            fig.plot_trajectory(path_planner, drones ,ta.drone_num)  
        fig.plot_all_targets()
        fig.plot_history(ta.optim.history)
        fig.show()
        fc.sleep()
        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
    # -------------------------------- Return all drones to base ------------------------#
    all_at_base = True
    for j in range(ta.drone_num):
        if not drones[j].at_base:
            all_at_base = False
    while not all_at_base:
        print('return all drones to base')
        for j in range(ta.drone_num):
            if not (drones[j].at_base) and not (drones[j].path_found) and (not (fc.open_threads[j].is_alive())):
                drones[j].start_coords = tuple(fc.get_position(j))
                drones[j].goal_title = 'base'
                drones[j].goal_coords = drones[j].base
                drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                if drones[j].path_found:
                    fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                        
            elif (drones[j].is_reached_goal):
                drones[j].at_base = 1
                
            drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords, title=drones[j].goal_title) 

                  
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
    


