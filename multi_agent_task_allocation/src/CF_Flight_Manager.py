import time
import cflib.crtp
import numpy as np
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.Ori_CF.fly_CF.Trajectory import Generate_Trajectory, upload_trajectory
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
        self.open_threads = [[]] * self.drone_num
        self.sleep_time = params.sleep_time
       
    def activate_high_level_commander(self, scf):
        scf.cf.param.set_value('commander.enHighLevel', '1')

    def wait_all_threads_killed(self):
        thread_alive = True
        while thread_alive:
            thread_alive = False
            for thread in self.open_threads:
                if thread.is_alive():
                    thread_alive = True
            time.sleep(1)
            print('-- waiting until all threads are killed')
        print('all threads dead')       

    def _take_off(self, scf):
        cf = scf.cf
        commander = cf.high_level_commander
        commander.takeoff(params.take_off_height, 2.0)
        time.sleep(3.0) 
    
    def take_off_swarm(self):
        threads = self.swarm.parallel_safe(self._take_off)
        for i in range(len(threads)):
            self.open_threads[i] = threads[i]
        self.wait_all_threads_killed()
        threads = self.swarm.parallel_safe(self._go_to_base)
        for i in range(len(threads)):
            self.open_threads[i] = threads[i]
        self.wait_all_threads_killed()

    def _go_to_base(self, scf):
        cf = scf.cf
        commander = cf.high_level_commander
        x,y,z = self.base[self.reversed_uri_dict[scf.cf.link_uri]]
        commander.go_to(x, y, z, yaw=0, duration_s=3)
        time.sleep(3)

    def _execute_trajectory(self, scf, waypoints): 
        cf = scf.cf
        commander = cf.high_level_commander 
        # x, y, z = waypoints[0]
        # print('start wp = ', waypoints[0])
        # commander.go_to(x, y, z, yaw=0, duration_s=1)
        # time.sleep(1)
        if len(waypoints) > 30:
            waypoints1 = waypoints[0:30]
            waypoints2 = waypoints[30:]
            wp_list = [waypoints1, waypoints2]
        else:
            wp_list = [waypoints]

        try:
            for waypoints in wp_list:
                trajectory_id = 1
                traj = Generate_Trajectory(waypoints, velocity=1, plotting=0, force_zero_yaw=False, is_smoothen=True)
                traj_coef = traj.poly_coef
                duration = upload_trajectory(cf, trajectory_id ,traj_coef)
                commander.start_trajectory(trajectory_id, 1.0, False)
                time.sleep(duration)
        except:
            print('failed to execute trajectory')

    
    def execute_trajectory_mt(self, drone_idx, waypoints):# send trajectory with multi thread mode
        thread = self.swarm.daemon_process(self._execute_trajectory, self.uri_dict[drone_idx], waypoints)
        self.open_threads[drone_idx] = thread
    
    def get_position(self, drone_idx):
        scf = self.swarm._cfs[self.uri_dict[drone_idx]]
        self.swarm.get_single_cf_estimated_position(scf)

        
    def reached_goal(self, drone_idx, goal):
        try:
            self.get_position(drone_idx)
            current_x, current_y, current_z = self.swarm._positions[self.uri_dict[drone_idx]]
            dist2goal = ((current_x - goal[0])**2 + (current_y - goal[1])**2 +(current_z - goal[2])**2 )**0.5
            print(f'distance to goal of drone {drone_idx} is : {dist2goal}')
            if dist2goal < 0.3:
                return 1
            else:
                return 0
        except:
            return 0

    def _land(self, scf):
        cf = scf.cf
        commander = cf.high_level_commander
        commander.land(0.0, 4.0)
        time.sleep(4)
        commander.stop()
    
    def land(self, drones): # complete - land only active drones
        self.wait_all_threads_killed()
        self.swarm.parallel_safe(self._land)
        self.swarm.close_links()
    
    def sleep(self):
        time.sleep(self.sleep_time)




