#! /usr/bin/env python3
# full simulation works for ros noetic on ubuntu 20.04 , python 3.8.10
# ------------------------ how to run  -----------------------
# terminal 1 : $ roslaunch rotors_gazebo drone_poll_circle.launch 
# terminal 2: $ rosrun multi_agent_task_allocation main_MATA_dw.py

# -------------------------------------------------------------

from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure , Drone_Manager, Logger
import numpy as np
import params
import time
plt.ion()

if not params.downwash_aware:
    raise Exception("downwash aware must be True")

if params.mode == 'sim':
    from rotors_flight_manager import Flight_manager
    import rospy
    rospy.init_node('send_my_command', anonymous=True)
    rospy.sleep(3)
elif params.mode == 'cf':
    from CF_Flight_Manager import Flight_manager

def main():
    start_time = time.time()
    logger = Logger('task_logger')
    ta = Allocation(logger) # compute fisrt targets allocation in init
    fig = get_figure()
    dm = Drone_Manager(params.uri_list, params.base, params.magazine, ta)
    fc = Flight_manager(ta.drone_num)
    path_planner = Trajectory(dm.drones, logger)
    fc.take_off_swarm()

    allocation = None
    last_unvisited = 0 #used for logging
    last_targets = 0 # used for logging

    while ta.optim.unvisited_num > 0:
        # ------------------------Target Allocation-------------------------- #   
        if ta.optim.unvisited_num != last_unvisited:
            logger.log(f'unvisited targets = {ta.optim.unvisited_num}')
            last_unvisited = ta.optim.unvisited_num
        for j in range(ta.drone_num):
            if dm.drones[j].is_available:
                change_flag = np.zeros(ta.drone_num, dtype=int)
                change_flag[j] = 1
                allocation = ta.allocate(change_flag)
                if allocation == 'update_kmeans':
                    logger.log(f'kmeans mode started, j = {j}')
                    break
                else:
                    dm.drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                    dm.drones[j].is_available = 0
        if not np.array_equal(last_targets, ta.optim.current_targets):
            last_targets = ta.optim.current_targets
            logger.log(f'current targets: {ta.optim.current_targets}')
        # --------------------------- UPDATE KMEANS ------------------------- #  
        while allocation == 'update_kmeans':
            k_means_permit = False
            while not k_means_permit:
                k_means_permit = dm.is_kmeas_permit(ta)
                for j in range(ta.drone_num):
                    # drones at base or target and available
                    if dm.drones[j].is_available:
                        continue
                    else:
                        dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal=dm.drones[j].goal_coords, title=dm.drones[j].goal_title) 
                        # find path to unvisited target
                        if (not (dm.drones[j].path_found)) and (dm.drones[j].goal_title == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True) and (not (fc.open_threads[j].is_alive())):
                            dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                            if dm.drones[j].path_found:
                                dm.drones[j].at_base = 0
                                fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                                logger.log(f'executing trajectory drone {j}')
                            else:
                                if dm.drones[j].at_base:
                                    dm.drones[j].is_available = 1
                                    logger.log(f'kmeans - drone {j} cant reach target, stay at base')
                                else:
                                    dm.drones[j].goal_title = 'base'
                                    dm.drones[j].goal_coords = dm.drones[j].base
                                    logger.log(f'kmeans - drone {j} cant reach target, returning to base')

                        # arrived to target
                        elif  (dm.drones[j].goal_title == 'target') and (dm.drones[j].is_reached_goal) :
                            dm.kmean_arrived_target(j, fc, ta)
                            logger.log(f'drone {j} arrived to target')
                            
                        # find path to base 
                        elif (not (dm.drones[j].path_found)) and dm.drones[j].goal_title == 'base'  and (not (fc.open_threads[j].is_alive())) :
                            dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                            if dm.drones[j].path_found:
                                fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                     
                        # arrived to base 
                        elif ((dm.drones[j].goal_title == 'base') and (dm.drones[j].is_reached_goal)):
                            dm.drones[j].at_base = 1
                            dm.drones[j].is_available = 1
                            logger.log(f'drone {j} arrived to base')

                fig.plot1(path_planner, dm, ta)
                fc.sleep()
            logger.log(f'kmeans permit {k_means_permit}')
            if k_means_permit :
                for j in range(ta.drone_num):
                   dm.kmeans_permit(j, fc)
                current_drone_num = ta.drone_num
                ta.update_kmeans(dm)
                for j in range(ta.drone_num):
                    dm.drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                    dm.drones[j].is_available = 0
                logger.log(f'kmeans updated, current drone num: {ta.drone_num}')
                logger.log(f'current targets: {ta.optim.current_targets}')
                allocation = None 

                #land inactive drones
                if current_drone_num > ta.drone_num: 
                    logger.log('returning to base inactive drones')
                    return2base = True
                    while return2base:
                        for j in range(current_drone_num-1, ta.drone_num-1,-1):
                            if not (dm.drones[j].at_base) and not (dm.drones[j].path_found) and (not (fc.open_threads[j].is_alive())):
                                dm.return_base(j, path_planner, fc, ta)
                            elif (dm.drones[j].is_reached_goal):
                                dm.drones[j].at_base = 1
                            dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal=dm.drones[j].goal_coords, title=dm.drones[j].goal_title)    
                        return2base = False
                        for j in range(current_drone_num-1, ta.drone_num-1,-1):
                            if not dm.drones[j].is_reached_goal:
                                return2base = True
                        fc.sleep()

                    for j in range(current_drone_num-1, ta.drone_num-1, -1):
                        fc.land(drone_idx=j)
                        dm.drones[j].is_active = False
                        logger.log(f'drone {j} is landing')
           
        #  -------------------------------- PATH PLANNING ------------------------------------------ #
        fig.ax.axes.clear()
        for j in range(ta.drone_num):
            if not (dm.drones[j].path_found) and (ta.optim.unvisited_num > 0) and (not (fc.open_threads[j].is_alive())):
                dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                if dm.drones[j].path_found:
                    dm.drones[j].at_base = 0
                    fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                    logger.log(f'executing trajectory drone {j}')
                else:
                    if not dm.drones[j].at_base:
                        dm.drones[j].goal_title = 'base'
                        dm.drones[j].goal_coords = dm.drones[j].base
                        logger.log(f'drone {j} cant reach target, returning to base')
    
            # ------------------ UPDATE DRONES STATUS -------------------------------------------#
            else:
                dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = dm.drones[j].goal_coords, title=dm.drones[j].goal_title) 
                if (dm.drones[j].is_reached_goal) and (dm.drones[j].path_found):
                    if dm.drones[j].goal_title == 'base':
                        dm.arrived_base(j, fc)
                        logger.log(f'drone {j} arrived to base')
                    elif dm.drones[j].goal_title == 'target':
                        dm.arrived_target(j, ta, fc)
                        logger.log(f'drone {j} arrived to target')
        
        fig.plot1(path_planner, dm, ta)
        fc.sleep()
    # -------------------------------- Return all drones to base ------------------------#
    all_at_base = dm.is_all_at_base(ta.drone_num)
    logger.log('return all drones to base')
    while not all_at_base:
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
    logger.log('landing all active drones')
    logger.log(f'Task Done Successfully, Total time:{round(time.time() - start_time, 2)} [sec]')


if __name__ == '__main__':
    main()
    


