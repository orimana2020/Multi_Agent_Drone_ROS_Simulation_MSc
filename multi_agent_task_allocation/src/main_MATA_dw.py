#! /usr/bin/env python3

# ------------------------ how to run  simulation-----------------------
# terminal 1 : $ roslaunch rotors_gazebo experiment1.launch 
# terminal 1 : $ roslaunch rotors_gazebo cf_exp.launch 

# terminal 2: $ rosrun multi_agent_task_allocation main_MATA_dw.py
# -------------------------------------------------------------

from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure , Drone_Manager, Logger, Analysis
import numpy as np
import params
import time
plt.ion()

def main():
    logger = Logger()
    an = Analysis()
    ta = Allocation(logger, an) # compute fisrt targets allocation in init
    fig = get_figure()
    dm = Drone_Manager(params.uri_list, params.base, params.magazine, ta)
    ta.update_kmeans(dm);  dm.update_first_goals(ta);
    fc = Flight_manager(ta.drone_num)
    path_planner = Trajectory(dm.drones, logger, an)
    fc.take_off_swarm()
    dm.update_current_coords(fc)
    an.start(dm)
    allocation = 'allocate'
    last_unvisited, last_targets = 0, 0 #used for logging

    while ta.optim.unvisited_num > 0:
        # ------------------------Target Allocation-------------------------- #   
        if ta.optim.unvisited_num != last_unvisited:
            logger.log(f'unvisited targets = {ta.optim.unvisited_num}')
            last_unvisited = ta.optim.unvisited_num
        if allocation == 'allocate':
            for j in range(ta.drone_num):
                if dm.drones[j].is_available:
                    change_flag = np.zeros(ta.drone_num, dtype=int)
                    change_flag[j] = 1
                    allocation = ta.allocate(change_flag)
                    if allocation == 'update_kmeans':
                        logger.log(f'kmeans mode started, j = {j}')
                        break
                    elif allocation == 'remove_drone':
                        logger.log(f'remove_drone started, j = {j}')
                        break
                    else:
                        dm.drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                        dm.drones[j].is_available = 0
        if not np.array_equal(last_targets, ta.optim.current_targets):
            last_targets = ta.optim.current_targets
            logger.log(f'current targets: {ta.optim.current_targets}')
        # --------------------------- REMOVE DRONES ------------------------- #  
        if allocation == 'remove_drone':    
            logger.log('returning to base inactive drones')
            drone_idx = ta.drone_num -1
            while not (dm.drones[drone_idx].at_base) and (ta.optim.unvisited[ta.optim.current_targets[drone_idx]]) and (dm.drones[drone_idx].path_found):
                dm.drones[drone_idx].is_reached_goal = fc.reached_goal(drone_idx=drone_idx, goal=dm.drones[drone_idx].goal_coords, title=dm.drones[drone_idx].goal_title)    
                if dm.drones[drone_idx].is_reached_goal and dm.drones[drone_idx].goal_title == 'target':
                    dm.kmean_arrived_target(drone_idx, fc, ta)
                    logger.log(f'drone {drone_idx} arrived to target')
                    an.time_to_target(drone_idx)
                    an.add_visited(drone_idx, ta.optim.current_targets[drone_idx])
                elif dm.drones[drone_idx].is_reached_goal and dm.drones[drone_idx].goal_title == 'base':
                    dm.drones[drone_idx].at_base = 1
                    logger.log(f'drone {j} arrived to base')
                    an.time_to_base(drone_idx)
                fc.sleep()

            while not (dm.drones[drone_idx].at_base):
                if not (dm.drones[drone_idx].path_found) : #and (not (fc.open_threads[drone_idx].is_alive())):
                    dm.return_base(drone_idx, path_planner, fc, ta, override=True)
                elif (dm.drones[drone_idx].is_reached_goal):
                    dm.drones[drone_idx].at_base = 1
                dm.drones[drone_idx].is_reached_goal = fc.reached_goal(drone_idx=drone_idx, goal=dm.drones[drone_idx].goal_coords, title=dm.drones[drone_idx].goal_title)    
                fc.sleep()
            
            allocation = 'update_kmeans'
            fc.land(drone_idx=drone_idx)
            dm.drones[drone_idx].is_active = False
            logger.log(f'drone {drone_idx} is landing')
            ta.drone_num -= 1
            logger.log(f'drone number updated to: {ta.drone_num}') 
            
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
                        if (not (dm.drones[j].path_found)) and (dm.drones[j].goal_title == 'target') and (ta.optim.unvisited[ta.optim.current_targets[j]] == True) : # and (not (fc.open_threads[j].is_alive())):
                            dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                            if dm.drones[j].path_found:
                                dm.drones[j].at_base = 0
                                an.atBaseTarget(j)
                                fc.execute_trajectory(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
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
                        elif  (dm.drones[j].goal_title == 'target') and (dm.drones[j].is_reached_goal):
                            dm.kmean_arrived_target(j, fc, ta)
                            logger.log(f'drone {j} arrived to target')
                            an.time_to_target(j)
                            an.add_visited(j, ta.optim.current_targets[j])
                            
                        # find path to base 
                        elif (not (dm.drones[j].path_found)) and dm.drones[j].goal_title == 'base'  :#and (not (fc.open_threads[j].is_alive())) :
                            dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                            if dm.drones[j].path_found:
                                fc.execute_trajectory(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                                an.atBaseTarget(j)
                     
                        # arrived to base 
                        elif ((dm.drones[j].goal_title == 'base') and (dm.drones[j].is_reached_goal)):
                            dm.drones[j].at_base = 1
                            dm.drones[j].is_available = 1
                            logger.log(f'drone {j} arrived to base')
                            an.time_to_base(j)

                fig.plot1(path_planner, dm, ta)
                fc.sleep()

            logger.log(f'kmeans permit {k_means_permit}')
            if (ta.optim.unvisited_num < ta.drone_num) and (ta.drone_num > 1):
                ta.drone_num_changed = True
                allocation = 'remove_drone'
                break

            if k_means_permit:
                for j in range(ta.drone_num):
                   dm.kmeans_permit(j, fc)
                ta.update_kmeans(dm)
                for j in range(ta.drone_num):
                    dm.drones[j].goal_coords = tuple(ta.targetpos[ta.optim.current_targets[j],:])
                    dm.drones[j].is_available = 0
                logger.log(f'kmeans updated, current drone num: {ta.drone_num}')
                logger.log(f'current targets: {ta.optim.current_targets}')
                allocation = 'allocate'
            


        #  -------------------------------- PATH PLANNING ------------------------------------------ #
        fig.ax.axes.clear()
        if allocation == 'allocate':
            for j in range(ta.drone_num):
                if not (dm.drones[j].path_found) and (ta.optim.unvisited_num > 0) : # and (not (fc.open_threads[j].is_alive())):
                    dm.drones[j].path_found = path_planner.plan(dm.drones ,drone_idx=j, drone_num=ta.drone_num)
                    if dm.drones[j].path_found:
                        dm.drones[j].at_base = 0
                        fc.execute_trajectory(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                        an.atBaseTarget(j)
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
                            an.time_to_base(j)
                            logger.log(f'drone {j} arrived to base')
                        elif dm.drones[j].goal_title == 'target':
                            an.time_to_target(j)
                            an.add_visited(j, ta.optim.current_targets[j])
                            dm.arrived_target(j, ta, fc)
                            logger.log(f'drone {j} arrived to target')
        if ((an.initial_targets_num - ta.optim.unvisited_num) / an.initial_targets_num ) > an.save_checkpoints[an.current_cp_idx]:
            print(((an.initial_targets_num - ta.optim.unvisited_num) / an.initial_targets_num ))
            an.save(an.save_checkpoints[an.current_cp_idx])
            an.current_cp_idx += 1
        fig.plot1(path_planner, dm, ta)
        fc.sleep()
    # -------------------------------- Return all drones to base ------------------------#
    all_at_base = dm.is_all_at_base(ta.drone_num)
    logger.log('return all drones to base')
    while not all_at_base:
        for j in range(ta.drone_num):
            if not (dm.drones[j].at_base) and not (dm.drones[j].path_found) : # and (not (fc.open_threads[j].is_alive())):
                dm.return_base(j, path_planner, fc, ta)
            elif (dm.drones[j].is_reached_goal):
                dm.drones[j].at_base = 1
            dm.drones[j].is_reached_goal = fc.reached_goal(drone_idx=j, goal = dm.drones[j].goal_coords, title=dm.drones[j].goal_title)       
        all_at_base = dm.is_all_at_base(ta.drone_num)
        fig.plot1(path_planner, dm, ta)
        fc.sleep()
    logger.log(f'Task Done Successfully, Total time:{round(time.time() - an.start_time, 2)} [sec], exclude take off and landing')
    an.save(1)
    fc.land('all', dm.drones)
    logger.log('landing all active drones')
    

if __name__ == '__main__':
    if not params.DOWNWASH_AWARE:
        raise Exception("DOWNWASH AWARE MUST BE TRUE IN PARAMS")
    if params.mode == 'cf' and params.SIMULATE_LPS_ERROR:
        raise Exception("SIMULATE_LPS_ERROR VALID IN SIMULATION ONLY")

    if params.mode == 'sim':
        from rotors_flight_manager import Flight_manager
        import rospy
        rospy.init_node('send_my_command', anonymous=True)
        rospy.sleep(3)
    elif params.mode == 'cf':
        from CF_Flight_Manager import Flight_manager
    main()
