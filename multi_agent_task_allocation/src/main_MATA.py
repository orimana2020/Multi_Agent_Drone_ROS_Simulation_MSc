#! /usr/bin/env python3
# full simulation works for ros noetic on ubuntu 20.04 , python 3.8.10
# -------- how to run sim mode ------------>
# terminal 1 : $ roslaunch rotors_gazebo drone_poll_circle.launch 
# terminal 2: $ rosrun multi_agent_task_allocation main_MATA.py
# ----------------------------------------->

from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure , Drone_Manager
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
    dm = Drone_Manager(params.uri_list, params.base, params.magazine, ta)
    fc = Flight_manager(ta.drone_num)
    path_planner = Trajectory(dm.drones)
    fc.take_off_swarm()
    allocation = None

    while ta.optim.unvisited_num > 0:
        # ------------------------Allocate targets -------- #   
        print(f'unvisited targets = {ta.optim.unvisited_num}') 
        for j in range(ta.drone_num):
            if dm.drones[j].is_available:
                change_flag = np.zeros(ta.drone_num, dtype=int)
                change_flag[j] = 1
                allocation = ta.allocate(change_flag)
                if allocation == 'update_kmeans':
                    print(f'kmeans mode started, j = {j}')
                    break
                else:
                    dm.drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                    dm.drones[j].is_available = 0
        # --------------------------- UPDATE KMEANS ------------------------- #  
        while allocation == 'update_kmeans':
            k_means_permit = False
            while not k_means_permit:
                k_means_permit = True
                for j in range(ta.drone_num):
                    if (not dm.drones[j].at_base ) or (ta.optim.unvisited[ta.optim.current_targets[j]] == True) :
                        k_means_permit = False
                print(f'kmeans permit {k_means_permit}')
                for j in range(ta.drone_num):
                    if (dm.drones[j].goal_coords == None) and (dm.drones[j].start_title == 'base'):
                        continue
                    elif (dm.drones[j].goal_coords == None) and dm.drones[j].start_title == 'target':
                        dm.kmeans_change_goal2base(j)
                    else:
                        dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal=dm.drones[j].goal_coords, title=dm.drones[j].goal_title) 
                    # find path to unvisited target
                    if (not (dm.drones[j].path_found)) and (dm.drones[j].goal_title == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True) and (not (fc.open_threads[j].is_alive())):
                        dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                        if dm.drones[j].path_found:
                            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])

                    # arrived to target
                    elif  (dm.drones[j].goal_title == 'target') and (dm.drones[j].is_reached_goal) :
                        dm.kmean_arrived_target(j, fc, ta)
                        
                    # find path to base / intermidiate
                    elif (not (dm.drones[j].path_found)) and dm.drones[j].goal_title == 'base'  and (not (fc.open_threads[j].is_alive())) :
                        dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                        if dm.drones[j].path_found:
                            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])

                    # arrived to base / intermidiate
                    elif ((dm.drones[j].goal_title == 'base') and (dm.drones[j].is_reached_goal)):
                        dm.drones[j].at_base = 1
                        dm.drones[j].goal_coords = None
                        dm.drones[j].start_title = 'base'

                    fig.plot1(path_planner, dm, ta)
                    fc.sleep()

            if k_means_permit :
                for j in range(ta.drone_num):
                   dm.kmeans_permit(j, fc)
                current_drone_num = ta.drone_num
                ta.update_kmeans()
                for j in range(ta.drone_num):
                    dm.drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                print('kmeans updated')
                allocation = None 
                if current_drone_num > ta.drone_num: #land inactive drones
                    for j in range(current_drone_num-1, ta.drone_num-1,-1):
                        fc.land(drone_idx=j)
                        dm.drones[j].is_active = False
                        print(f'drone {j} is landing')
           
        #  -------------------------------- PATH PLANNING ------------------------------------------ #
        fig.ax.axes.clear()
        for j in range(ta.drone_num):
            if not (dm.drones[j].path_found) and (ta.optim.unvisited_num > 0) and (not (fc.open_threads[j].is_alive())):
                dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                if dm.drones[j].path_found:
                    dm.drones[j].at_base = 0
                    fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
    
            # ------------------ UPDATE DRONES STATUS -------------------------------------------#
            else:
                dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = dm.drones[j].goal_coords, title=dm.drones[j].goal_title) 
                if (dm.drones[j].is_reached_goal) and (dm.drones[j].path_found):
                    if dm.drones[j].goal_title == 'base':
                        dm.arrived_base(j, fc)
                    elif dm.drones[j].goal_title == 'target':
                        dm.arrived_target(j, ta, fc)
        
        fig.plot1(path_planner, dm, ta)
        fc.sleep()
        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
    # -------------------------------- Return all drones to base ------------------------#
    all_at_base = dm.is_all_at_base(ta.drone_num)
    while not all_at_base:
        print('return all drones to base')
        for j in range(ta.drone_num):
            if not (dm.drones[j].at_base) and not (dm.drones[j].path_found) and (not (fc.open_threads[j].is_alive())):
                dm.return_base(j, path_planner, fc, ta)
            elif (dm.drones[j].is_reached_goal):
                dm.drones[j].at_base = 1
            dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = dm.drones[j].goal_coords, title=dm.drones[j].goal_title)       
        all_at_base = dm.is_all_at_base(ta.drone_num)
        fig.plot1(path_planner, dm, ta)
        fc.sleep()
    fc.land('all', dm.drones)
    print(' Task Done Successfully')


if __name__ == '__main__':
    main()
    


