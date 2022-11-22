#! /usr/bin/env python
import numpy as np
from sklearn.cluster import KMeans
from itertools import combinations
import random
import os
from itertools import permutations
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.ion()

class Env(object):
    def __init__(self, drone_performace, drone_num):
        self.drone_num = drone_num
        self.drone_peformace = np.zeros([self.drone_num, 100])
        for i in range(self.drone_num):
            self.drone_peformace[i,0:drone_performace[i]] = 1
    
    def reached_goal(self, drone_idx, goal=None): #in real simulation reached_goal is given if distance between drone and goal below some value
        if self.drone_peformace[drone_idx, random.randint(0,99)] == 1:
            return 1
        return 0



class Drone(object):
    
    def __init__(self, drone_num, safety_distance, magazine):
        self.drone_num = drone_num
        self.safety_distance = safety_distance
        self.drone_num_changed = 0
        self.colors = ['r', 'g', 'b', 'peru', 'yellow', 'lime', 'navy', 'purple', 'pink','grey']
        self.base = [(1.5,0,1),(1.5,0.7,1),(1.5,-0.7,1),(-1,0.2,1),(-1,0.2,1)] # (x,y,z)
        self.full_magazine = magazine[:]
        self.current_magazine = magazine[:]
        self.start_title = []
        self.goal_title = []
        for _ in range(drone_num):
            self.start_title.append('base')
            self.goal_title.append('target')



class Targets(object):

    def __init__(self, targets_num, data_source):
        self.targets_num = targets_num
        t = np.linspace(0, 2*np.pi-2*np.pi/targets_num, targets_num)
        radius = 0.7
        depth = 3.5 
        z_offset = radius + 1
        if data_source == 'circle':
            self.targetpos = np.stack([depth*np.ones([targets_num]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)
        elif data_source == 'dataset':
            # self.targetpos = np.load(str(os.getcwd())+'/src/drone_pollination/src/targets_arr.npy')
            self.targetpos = np.load(str(os.getcwd())+'/src/rotors_simulator/multi_agent_task_allocation/src/targets_arr.npy')
        self.targets_num,_ = self.targetpos.shape

        self.targetpos_reallocate = self.targetpos.copy()
        self.limits = self.set_limits()
        self.span = [self.z_span, self.y_span, self.x_span]

    def set_limits(self):
        self.z_min, self.z_max = 0, max(self.targetpos[:,2])
        self.z_span = (self.z_max - self.z_min) * 1.3
        self.y_min, self.y_max =  min(self.targetpos[:,1]), max(self.targetpos[:,1])
        self.y_span = (self.y_max - self.y_min) * 1.3
        self.x_min, self.x_max = 0, max(self.targetpos[:,0])
        self.x_span = (self.x_max - self.x_min) * 1.3        
            



        
class Optim(object):
    def __init__(self, targets, drone, k):
        self.k = k
        # calc distance matrix
        self.distance_mat = np.zeros([targets.targets_num, targets.targets_num])
        for i in range(targets.targets_num):
            tar1 = targets.targetpos[i,:]
            for j in range(targets.targets_num):
                tar2 = targets.targetpos[j,:]
                self.distance_mat[i,j] = np.linalg.norm(tar1 - tar2, ord=2)
                if i == j:
                    self.distance_mat[i,j] = np.Inf
        
        self.distance_mat_nochange = self.distance_mat.copy()
        self.unvisited_num = targets.targets_num
        self.current_targets = np.zeros(drone.drone_num, dtype=int)
        self.min_dist_step = [] #for debuging algorithm performance
        self.unvisited = np.ones(targets.targets_num, dtype=bool)
        self.threshold_factor = 0.8

        # History
        self.history =[]
        for i in range(drone.drone_num):
            self.history.append(np.empty((0,3)))


    def get_state_matrix(self, drone_num, init_flag=False):
        if init_flag:
            return np.load(str(os.getcwd())+'/src/rotors_simulator/multi_agent_task_allocation/src'+'/state_mat/state_mat_d'+str(drone_num)+'_k'+str(self.k)+'.npy')
        else:   
            if self.k > 1:
                self.k -= 1
            print('k updated:' , self.k)
            if self.k >=2:
                return np.load(str(os.getcwd())+'/src/rotors_simulator/multi_agent_task_allocation/src'+'/state_mat/state_mat_d'+str(drone_num)+'_k'+str(self.k)+'.npy')
            elif self.k == 1:
                return np.ones((1,drone_num,1), dtype=int)
    


    def get_knn(self, is_changed, drone_num):
        temp_distance_matrix = self.distance_mat.copy()
        # set columns of current targets to inf so they will not be selected again
        for i in range(drone_num):
            temp_distance_matrix[:, self.current_targets[i]] = np.Inf
        
        # find k nearest targets
        knn = np.zeros([self.k, drone_num], dtype=int)
        for i in range(drone_num):
            if is_changed[i] == 1:
                knn[:, i] = np.argsort(temp_distance_matrix[self.current_targets[i], :])[:self.k] 
                for idx in knn[:,i]:
                    temp_distance_matrix[:,idx] = np.inf
            else:
                knn[:,i] = self.current_targets[i]      
        return knn



    def search_best_combination(self, drone_num, state_mat, knn):
        min_diff = np.inf
        for i in range(self.k ** drone_num):
            next_comb = np.sum(knn * state_mat[:,:,i], axis=0)
            dist = np.zeros(self.combs_size)
            for j in range(self.combs_size):
                # dist[j] = self.distance_mat[next_comb[self.combs[j][0]], next_comb[self.combs[j][1]]]
                dist[j] = self.distance_mat_nochange[next_comb[self.combs[j][0]], next_comb[self.combs[j][1]]]
            next_diff = np.linalg.norm(self.initial_dist_vec - dist, ord=2)
            if next_diff < 0.001: # added due to numerical error caused skipped optimal allocation
                next_diff = 0
            if next_diff < min_diff:
                min_diff = next_diff
                min_dist = min(dist)
                best_comb = next_comb
                travel_dist = 0
                for j in range(drone_num):
                    travel_dist += self.distance_mat[self.current_targets[j], best_comb[j]]

            elif next_diff == min_diff:
                travel_dist2 = 0
                for j in range(drone_num):
                    travel_dist2 += self.distance_mat[self.current_targets[j], next_comb[j]]  
                if travel_dist2 < travel_dist:
                    best_comb = next_comb
                    min_dist = min(dist)
                    travel_dist = travel_dist2
        
        return best_comb, min_dist       
   


    def get_1_min(self, targets, test_pnt): #get k-smallest indeces (distance from kmeans centers to targets)
        diff = np.linalg.norm(targets - test_pnt, ord=2, axis=1)   
        return np.argmin(diff)  
    

    def update_history(self, target_idx, drone_idx, targets):
        self.history[drone_idx] = np.vstack((self.history[drone_idx], targets.targetpos[target_idx,:]))
    
    def update_distance_mat(self, target_idx):
        self.distance_mat[:, target_idx] = np.inf

    
    def  update_kmeans_drone_num(self, drone, targets):
        print('kmeans updated')
        first_itr = 1
        self.min_dist_candidate = drone.safety_distance

        while (self.min_dist_candidate < drone.safety_distance or first_itr == 1) and (drone.drone_num > 1):
            first_itr = 0
            #get initial target by Kmeans
            self.kmeans = KMeans(n_clusters=drone.drone_num).fit(targets.targetpos[self.unvisited,:])
            self.centers = self.kmeans.cluster_centers_
            self.current_targets = np.zeros(drone.drone_num, dtype=int)
            unvisited_targets_temp = targets.targetpos_reallocate.copy()
            for i in range(drone.drone_num): 
                self.current_targets[i] = self.get_1_min(unvisited_targets_temp, self.centers[i,:])
                unvisited_targets_temp[self.current_targets[i],:] = np.inf 
            
            # calc initial dist vec
            self.combs = list(combinations(list(range(drone.drone_num)), 2))
            self.combs_size = len(self.combs)
    
            self.initial_dist_vec = np.zeros(self.combs_size)
            for k in range(self.combs_size):
                i,j = self.combs[k]
                self.initial_dist_vec[k] = self.distance_mat[self.current_targets[i], self.current_targets[j]]  

            self.min_dist_candidate = min(self.initial_dist_vec)
            self.threshold_dist = self.threshold_factor * self.min_dist_candidate
            print('min_dist_org = ', self.min_dist_candidate)
            print('threshold dist = ', self.threshold_dist)

            if (self.min_dist_candidate < drone.safety_distance ) and (drone.drone_num > 1) :
                drone.drone_num -= 1
                drone.drone_num_changed = 1
                print('drones number updated: ', drone.drone_num)


    def get_knn_last_drone(self):
        temp_distance_matrix = self.distance_mat.copy()
        # set columns of current targets to inf so they will not be selected again
        temp_distance_matrix[:, self.current_targets[0]] = np.inf
        # find nearest target
        return np.argsort(temp_distance_matrix[self.current_targets[0], :])[0]
            

class get_figure(object):
    def __init__(self, targets, drone):
        self.targets = targets
        self.drone = drone
        self.fig = plt.figure(1)
        self.ax =  Axes3D(self.fig)
        self.ax.axes.set_xlim(left=targets.x_min, right=targets.x_max) 
        self.ax.axes.set_ylim(bottom=targets.y_min, top=targets.y_max) 
        self.ax.axes.set_zlim(bottom=targets.z_min, top=targets.z_max)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        # self.ax.view_init(elev=0, azim=90)
        self.ax.view_init(elev=37, azim=175)
    
    def plot_all_targets(self):
        self.ax.scatter3D(self.targets.targetpos[:,0], self.targets.targetpos[:,1], self.targets.targetpos[:,2], s= 10, c='k',alpha=1, depthshade=False)
    
    def plot_current_targets(self, current_idx, drone_num):
        for j in range(drone_num):
            self.ax.scatter3D(self.targets.targetpos[current_idx[j], 0], self.targets.targetpos[current_idx[j], 1], self.targets.targetpos[current_idx[j], 2], s =50, c=self.drone.colors[j], alpha=1,depthshade=False)

    def plot_trajectory(self, path_planner, path_found,drone_num,goal_title,path_scatter=0, smooth_path_cont=1,smooth_path_scatter=0, block_volume=0):
        for j in range(drone_num):
            if path_found[j] == 1:
                if path_scatter == 1:
                    self.ax.scatter3D(path_planner.paths_m[j][:,0], path_planner.paths_m[j][:,1], path_planner.paths_m[j][:,2], s= 15, c='r',alpha=1, depthshade=False)
                if smooth_path_cont == 1:  
                    if goal_title[j] =='base':
                        color = 'r' 
                    else:
                        color = 'b'
                    self.ax.plot(path_planner.smooth_path_m[j][:,0],path_planner.smooth_path_m[j][:,1],path_planner.smooth_path_m[j][:,2], c=color, linewidth=4)
                if smooth_path_scatter == 1:
                    self.ax.scatter3D(path_planner.smooth_path_m[j][:,0],path_planner.smooth_path_m[j][:,1],path_planner.smooth_path_m[j][:,2],s= 25, c='g',alpha=1, depthshade=False)
                if block_volume == 1:
                    self.ax.scatter3D(path_planner.block_volumes_m[j][:,0], path_planner.block_volumes_m[j][:,1], path_planner.block_volumes_m[j][:,2], s= 10, c='g',alpha=0.01,depthshade=False)

    
    def show(self, sleep_time):
        self.ax.axes.set_xlim(left=self.targets.x_min, right=self.targets.x_max) 
        self.ax.axes.set_ylim(bottom=self.targets.y_min, top=self.targets.y_max) 
        self.ax.axes.set_zlim(bottom=self.targets.z_min, top=self.targets.z_max)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.fig.canvas.flush_events()
        

    def plot_at_base(self, drone_num, at_base):
        for j in range(drone_num):
            if at_base[j] == 1:
                self.ax.scatter3D(self.drone.base[j][0], self.drone.base[j][1], self.drone.base[j][2], s=80 ,marker='x', c=self.drone.colors[j] ,depthshade=False)



    def plot_no_path_found(self, start, goal):
        coord = np.stack([np.array(start), np.array(goal)], axis=0)
        self.ax.plot(coord[:,0],coord[:,1],coord[:,2], c='m', linewidth=6)


    def plot_history(self, history, drone_num, colors):
        for j in range(drone_num):
            if len(history[j]) > 0:
                self.ax.scatter3D(history[j][:,0], history[j][:,1], history[j][:,2], s =50, c=colors[j], alpha=1,depthshade=False)
            
        
        

 







    