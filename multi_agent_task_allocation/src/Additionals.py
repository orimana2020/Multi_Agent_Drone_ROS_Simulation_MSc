#! /usr/bin/env python
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import params 

class Drone_Manager(object):
    def __init__(self, uris, base, full_magazine, ta):
        self.drones = []
        for i in range(ta.drone_num):
            self.drones.append(Drone(index=i, uri=uris[i], base=base[i], full_magazine=full_magazine[i], ta=ta))
    
    def arrived_base(self,j, fc):
        self.drones[j].start_title = 'base'
        self.drones[j].start_coords = tuple(fc.get_position(j))
        self.drones[j].goal_title = 'target'
        self.drones[j].goal_coords = None
        self.drones[j].current_magazine = self.drones[j].full_magazine
        self.drones[j].at_base = 1
        self.drones[j].is_available = 1
        self.drones[j].path_found = 0
        self.drones[j].is_reached_goal = 0

    def arrived_target(self, j, ta, fc):
        self.drones[j].start_title = 'target'
        self.drones[j].start_coords = tuple(fc.get_position(j))
        self.drones[j].current_magazine -= 1
        self.drones[j].path_found = 0
        self.drones[j].is_reached_goal = 0
        self.drones[j].at_base = 0
        ta.optim.unvisited_num -= 1
        ta.optim.unvisited[ta.optim.current_targets[j]] = False
        ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
        ta.targetpos_reallocate[ta.optim.current_targets[j],:] = np.inf
        ta.optim.update_distance_mat(ta.optim.current_targets[j])
        if self.drones[j].current_magazine > 0:
            self.drones[j].is_available = 1
            self.drones[j].goal_title = 'target'
            self.drones[j].goal_coords = None
        else:
            self.drones[j].is_available = 0   
            self.drones[j].goal_title = 'base'
            self.drones[j].goal_coords = self.drones[j].base

    def kmean_arrived_target(self,j, fc, ta):
        ta.optim.unvisited_num -= 1
        ta.optim.unvisited[ta.optim.current_targets[j]] = False
        ta.optim.update_history(ta.optim.current_targets[j], j, ta.targetpos) 
        ta.targetpos_reallocate[ta.optim.current_targets[j], :] = np.inf
        ta.optim.update_distance_mat(ta.optim.current_targets[j])
        self.drones[j].path_found = 0
        self.drones[j].start_title = 'target' 
        self.drones[j].start_coords = tuple(fc.get_position(j))
        self.drones[j].is_reached_goal = 0 
        self.drones[j].current_magazine -= 1
        self.drones[j].at_base = 0 
        if self.drones[j].current_magazine > 0:
            self.drones[j].is_available = 1
            self.drones[j].goal_title = 'target'
            self.drones[j].goal_coords = None
        else:
            self.drones[j].goal_title = 'base'
            self.drones[j].goal_coords = self.drones[j].base
            self.drones[j].is_available = 0


    def kmeans_permit(self, j, fc):
        if self.drones[j].at_base:
            self.drones[j].start_title = 'base'
            self.drones[j].start_coords = tuple(fc.get_position(j))
            self.drones[j].current_magazine = self.drones[j].full_magazine
            self.drones[j].goal_title = 'target'
            self.drones[j].is_reached_goal = 0
            self.drones[j].path_found = 0 
            self.drones[j].is_available = 0
    
    def is_kmeas_permit(self, ta):
        k_means_permit = True
        for j in range(ta.drone_num):
            if (not self.drones[j].is_available) :
                k_means_permit = False
        return k_means_permit

    def return_base(self, j, path_planner, fc, ta):
        self.drones[j].start_coords = tuple(fc.get_position(j))
        self.drones[j].goal_title = 'base'
        self.drones[j].goal_coords = self.drones[j].base
        self.drones[j].path_found = path_planner.plan(self.drones ,drone_idx=j, drone_num=ta.drone_num)
        if self.drones[j].path_found:
            fc.execute_trajectory_mt(drone_idx=j, waypoints=path_planner.smooth_path_m[j])
                
    
    def is_all_at_base(self, drone_num):
        all_at_base = True
        for j in range(drone_num):
            if not self.drones[j].at_base:
                all_at_base = False
        return all_at_base
    


class Drone(object):
    def __init__(self, index, uri, base, full_magazine, ta):
        self.idx = index
        self.uri = uri
        self.base = base
        self.start_coords = base
        self.goal_coords = tuple(ta.targetpos[ta.optim.current_targets[self.idx],:])
        self.full_magazine = full_magazine
        self.current_magazine = full_magazine
        self.start_title = 'base'
        self.goal_title = 'target'
        self.is_available = 0
        self.is_reached_goal = 0
        self.path_found = 0
        self.at_base = 1
        self.is_active = True
        self.battery = None
        self.path_idx = None
        self.path_m = None
        self.current_target = None
        self.block_volume_idx = None
        self.block_volume_base = None
        self.block_volume_m = None
    
        
                
class get_figure(object):
    def __init__(self):
        self.targetpos = params.targetpos
        self.inital_drone_num = params.drone_num
        self.colors = params.colors
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max = params.limits
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        # self.ax.view_init(elev=0, azim=90)
        elev, azim = params.elvazim
        self.ax.view_init(elev=elev, azim=azim)
        self.path_scatter = params.plot_path_scatter
        self.smooth_path_cont = params.plot_smooth_path_cont 
        self.smooth_path_scatter = params.plot_smooth_path_scatter
        self.block_volume = params.plot_block_volume
        self.constant_blocking_area = params.plot_constant_blocking_area
        self.plot_block_volume_floor_m = params.plot_block_volume_floor_m
    
    def plot_all_targets(self):
        self.ax.scatter3D(self.targetpos[:,0], self.targetpos[:,1], self.targetpos[:,2], s= 10, c='k',alpha=1, depthshade=False)
    
    def plot_current_targets(self, current_idx, drone_num):
        for j in range(drone_num):
            self.ax.scatter3D(self.targetpos[current_idx[j], 0], self.targetpos[current_idx[j], 1], self.targetpos[current_idx[j], 2], s =50, c=self.colors[j], alpha=1,depthshade=False)

    def plot_trajectory(self, path_planner, drones ,drone_num):
        for j in range(drone_num):
            if drones[j].path_found:
                if self.path_scatter:
                    self.ax.scatter3D(path_planner.paths_m[j][:,0], path_planner.paths_m[j][:,1], path_planner.paths_m[j][:,2], s= 15, c='r',alpha=1, depthshade=False)
                if self.smooth_path_cont:  
                    if drones[j].goal_title =='base':
                        color = 'r' 
                    else:
                        color = 'b'
                    self.ax.plot(path_planner.smooth_path_m[j][:,0],path_planner.smooth_path_m[j][:,1],path_planner.smooth_path_m[j][:,2], c=color, linewidth=4)
                if self.smooth_path_scatter:
                    self.ax.scatter3D(path_planner.smooth_path_m[j][:,0],path_planner.smooth_path_m[j][:,1],path_planner.smooth_path_m[j][:,2],s= 25, c='g',alpha=1, depthshade=False)
                if self.block_volume:
                    self.ax.scatter3D(path_planner.block_volumes_m[j][:,0], path_planner.block_volumes_m[j][:,1], path_planner.block_volumes_m[j][:,2], s= 10, c='g',alpha=0.1,depthshade=False)
                if self.constant_blocking_area:
                    self.ax.scatter3D(path_planner.constant_blocking_area_m[j][:,0], path_planner.constant_blocking_area_m[j][:,1], path_planner.constant_blocking_area_m[j][:,2], s= 10, c='m',alpha=0.1,depthshade=False)
                if self.plot_block_volume_floor_m:
                    self.ax.plot(path_planner.block_volume_floor_m[:,0],path_planner.block_volume_floor_m[:,1], path_planner.block_volume_floor_m[:,2], c='grey', linewidth=4)
    def show(self):
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.ax.set_xlim3d([self.x_min, self.x_max])
        self.ax.set_ylim3d([self.y_min, self.y_max])
        self.ax.set_zlim3d([self.z_min, self.z_max])
        self.fig.canvas.flush_events()

    def plot_no_path_found(self, drone):
        no_path = np.stack([np.array(drone.start_coords), np.array(drone.goal_coords)], axis=0)
        self.ax.plot(no_path[:,0], no_path[:,1], no_path[:,2], c='m', linewidth=6)

    def plot_history(self, history):
        for j in range(self.inital_drone_num):
            if len(history[j]) > 0:
                self.ax.scatter3D(history[j][:,0], history[j][:,1], history[j][:,2], s =50, c=self.colors[j], alpha=1,depthshade=False)
            
    def plot1(self, path_planner, dm, ta):
        self.ax.axes.clear()
        self.plot_all_targets()
        self.plot_trajectory(path_planner, dm.drones ,ta.drone_num)
        self.plot_history(ta.optim.history)
        self.show()



def generate_fake_error_mapping(): 
    x_min,x_max,y_min,y_max,z_min,z_max = params.limits_idx
    y_max = round(y_max - y_min) 
    y_min = 0
    res = params.resolution
    worst_accuracy = 0.5 / res
    best_accuracy = 0.05 / res
    error_arr = np.zeros([z_max-z_min,y_max-y_min, x_max-x_min], dtype=int)
    y_middle = round((y_min+y_max)/2)
    for x in range(x_max):
        for y in range(y_max):
            for z in range(z_max):
                dist_x = x
                total_dist_score_x = (dist_x / x_max) 
                total_dist_score_y = (np.exp(abs(y-y_middle)/y_middle) - 1) / (np.exp(1) - 1)
                total_dist_score = (total_dist_score_x + total_dist_score_y) / 2
                error_arr[z,y,x] = round(total_dist_score * (worst_accuracy - best_accuracy) + best_accuracy)
    print(f'error arr shape: {error_arr.shape}')
    return error_arr


class Env(object):
    def __init__(self, drone_performace, drone_num):
        self.drone_num = drone_num
        self.drone_peformace = np.zeros([self.drone_num, 100])
        for i in range(self.drone_num):
            self.drone_peformace[i,0:drone_performace[i]] = 1
    def reached_goal(self, drone_idx, goal=None): 
        if self.drone_peformace[drone_idx, random.randint(0,99)] == 1:
            return 1
        return 0


    