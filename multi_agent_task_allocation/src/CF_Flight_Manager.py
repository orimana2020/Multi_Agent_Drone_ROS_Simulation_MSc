import time
import cflib.crtp
import numpy as np
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from Ori_CF.multi_agent_task_allocation.src.CF_Trajectory import Generate_Trajectory, upload_trajectory
import params

class Flight_manager(object):
    def __init__(self, drone_num):
        self.drone_num = drone_num
        self.uri_list = params.uri_list[:self.drone_num]
        self.base = params.base
        self.uri_dict = dict()
        self.reversed_uri_dict = dict()
        for i  in range(len(self.uri_list)):
            self.uri_dict[i] = self.uri_list[i]
            self.reversed_uri_dict[self.uri_list[i]] = i
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(set(self.uri_list), factory=factory)
        self.swarm.open_links()
        self.swarm.parallel_safe(self.activate_high_level_commander)
        self.swarm.reset_estimators()
        self.sleep_time = params.sleep_time
        self.smooth_points_num = params.segments_num + 1
        self.max_dist2target = params.dist_to_target
        self.max_dist2base = params.dist_to_base
        self.velocity = params.linear_velocity
       
    def activate_high_level_commander(self, scf):
        scf.cf.param.set_value('commander.enHighLevel', '1')



    def _take_off(self, scf):
        commander = scf.cf.high_level_commander
        commander.takeoff(params.take_off_height, 2.0)
        time.sleep(2.0) 
        
    def _go_to_base(self, scf):
        commander = scf.cf.high_level_commander
        x,y,z = self.base[self.reversed_uri_dict[scf.cf.link_uri]]
        commander.go_to(x, y, z, yaw=0, duration_s=2)
        time.sleep(2)

    def take_off_swarm(self):
        self.swarm.parallel_safe(self._take_off)
        self.swarm.parallel_safe(self._go_to_base)
    
    def execute_trajectory(self, drone_idx, waypoints): 
        scf = self.swarm._cfs[self.uri_dict[drone_idx]]
        cf = scf.cf
        commander = scf.cf.high_level_commander 
        velocity = [1.5, 0.8]
        if len(waypoints) > self.smooth_points_num:
            waypoints1 = waypoints[:self.smooth_points_num] # target to retreat
            waypoints2 = waypoints[self.smooth_points_num:] # retreat to target
            wp_list = [waypoints1, waypoints2]
        else:
            wp_list = [waypoints]
       
        try:
            for idx, waypoints in enumerate(wp_list):
                x, y, z = waypoints[0]
                commander.go_to(x, y, z, yaw=0, duration_s=0.1)
                trajectory_id = 1
                traj = Generate_Trajectory(waypoints, velocity=velocity[idx], force_zero_yaw=False)
                traj_coef = traj.poly_coef
                duration = upload_trajectory(cf, trajectory_id ,traj_coef)
                commander.start_trajectory(trajectory_id, 1.0, False)
                if idx == 0:
                    time.sleep(duration)
        except:
            print('failed to execute trajectory')


    def get_position(self, drone_idx):
        scf = self.swarm._cfs[self.uri_dict[drone_idx]]
        self.swarm._get_estimated_position(scf)
        return self.swarm._positions[self.uri_dict[drone_idx]]
     
        
    def reached_goal(self, drone_idx, goal, title='target'):
        try:
            self.get_position(drone_idx)
            current_x, current_y, current_z = self.swarm._positions[self.uri_dict[drone_idx]]
            dist2goal = ((current_x - goal[0])**2 + (current_y - goal[1])**2 +(current_z - goal[2])**2 )**0.5
            max_dist = self.max_dist2target if title == 'target' else self.max_dist2base
            if dist2goal < max_dist:
                print(f'drone {drone_idx} reached goal')
                return 1
            else:
                return 0
        except:
            return 0

    def _land(self, scf):
        commander = scf.cf.high_level_commander
        commander.land(0.0, 3)
        time.sleep(3)
        commander.stop()
        scf.close_link()
    
    def land(self, drone_idx, drones=None): 
        if drone_idx == 'all':
            for i in range(len(drones)):
                scf = self.swarm._cfs[self.uri_dict[i]]
                self._land(scf)
        else:
            scf = self.swarm._cfs[self.uri_dict[drone_idx]]
            self._land(scf)
            
        
    def sleep(self):
        time.sleep(self.sleep_time)




