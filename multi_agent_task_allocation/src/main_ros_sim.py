#! /usr/bin/env python3

# full simulation works for ros noetic on ubuntu 20.04
# python 3.8.10
# -------- how to run------------>
# terminal 1 : $ roslaunch rotors_gazebo drone_poll_lanch.launch  
# terminal 2: $ rosrun multi_agent_task_allocation main_ros_sim.py 


from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure ,Get_Drones
import numpy as np
import params

if params.mode == 'sim':
    from rotors_flight_manager import Flight_manager
    import rospy
    rospy.init_node('send_my_command', anonymous=True)
    rospy.sleep(3)
elif params.mode == 'cf':
    pass

plt.ion()

def main():
    ta = Allocation() 
    fig = get_figure()
    fc = Flight_manager()
    allocation = None
    drones = Get_Drones(params.uri_list, params.base, params.magazine, ta.drone_num)
    path_planner = Trajectory(ta.drone_num, drones)
    fc.take_off_swarm( height=params.take_off_height , drone_num=ta.drone_num)
    
    while ta.optim.unvisited_num > 0:
        print('unvisited = %d' %ta.optim.unvisited_num)
        # ------------------------     update magazine state & allocate new targets -------- #    
        for j in range(ta.drone_num):
            if drones[j].is_available:
                change_flag = np.zeros(ta.drone_num, dtype=int)
                change_flag[j] = 1
                allocation = ta.allocate(change_flag)
                if allocation == 'update_kmeans':
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
                drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = drones[j].goal_coords) 
                if not (drones[j].at_base):
                    # arrived to target
                    if  (drones[j].goal_title == 'target') and (drones[j].path_found) and (drones[j].is_reached_goal) and (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
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

                    # find path to target
                    if not (drones[j].path_found) and (drones[j].goal_title == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True):
                        drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                        if drones[j].path_found:
                            fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)
                    
                    # find path to base
                    if not (drones[j].path_found) and  (ta.optim.unvisited[ta.optim.current_targets[j]] == False) :
                        drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                        if drones[j].path_found:
                            fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)
                        if not drones[j].path_found:
                            fig.plot_no_path_found(drones[j].start_coords, drones[j].goal_coords)  

                    if ((drones[j].path_found) and (drones[j].goal_title == 'base') and (drones[j].is_reached_goal)):
                        drones[j].at_base = 1
                        
            fig.ax.axes.clear()
            fig.plot_all_targets()
            fig.plot_trajectory(path_planner, drones ,ta.drone_num)
            fig.plot_history(ta.optim.history)
            fig.show()

            k_means_permit = True
            for j in range(ta.drone_num):
                if not drones[j].at_base:
                    k_means_permit = False
            if k_means_permit :
                for j in range(ta.drone_num):
                    drones[j].current_magazine = drones[j].full_magazine
                    drones[j].start_title = 'base'
                    drones[j].goal_title = 'target'
                    drones[j].current_pos_title = 'base'
                    drones[j].current_pos_coords = drones[j].base
                    drones[j].is_reached_goal = 0
                    drones[j].path_found = 0 
                    drones[j].is_available = 0
                ta.update_kmeans()
                allocation = None  
            fc.sleep()
           
        #  --------------------------------    path planning ----------------------------- #
        fig.ax.axes.clear()
        for j in range(ta.drone_num):
            if not (drones[j].path_found) and (ta.optim.unvisited_num > 0): #force trying plan to base until is found
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
                    fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)

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
                if (drones[j].current_pos_title == 'target') and not (drones[j].path_found):
                    drones[j].start_title = drones[j].current_pos_title
                    drones[j].start_coords = drones[j].current_pos_coords
                    drones[j].goal_title = 'base'
                    drones[j].goal_coords = drones[j].base
                    drones[j].path_found = path_planner.plan(drones ,drone_idx=j, drone_num=ta.drone_num)
                    if drones[j].path_found:
                        fc.publish_traj_command(path_planner.smooth_path_m[j], drone_idx=j)

                if (drones[j].is_reached_goal) and (drones[j].path_found):
                    drones[j].at_base = 1
                    ta.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
                    ta.optim.update_distance_mat(ta.optim.current_targets[j])

        for j in range(ta.drone_num):
            if not drones[j].at_base:
                all_at_base = False
        fig.ax.axes.clear()
        fig.plot_history(ta.optim.history)
        fig.plot_all_targets()    
        fig.plot_trajectory(path_planner, drones ,ta.drone_num)
        fig.show()
        all_at_base = True
        fc.sleep()
    fig.plot_all_targets()
    fig.plot_history(ta.optim.history)
    fig.show()
    print(' Task Done Successfully')


if __name__ == '__main__':
    main()
    


