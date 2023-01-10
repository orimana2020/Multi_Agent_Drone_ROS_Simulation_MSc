#! /usr/bin/env python
import numpy as np
from itertools import permutations 
import os
from sklearn.cluster import KMeans
from itertools import combinations
import params

class Optim(object):
    def __init__(self, targets_num, targetpos, logger, an):
        drone_num = params.drone_num
        self.k = params.k_init
        # calc distance matrix
        self.distance_mat = np.zeros([targets_num, targets_num])
        for i in range(targets_num):
            tar1 = targetpos[i,:]
            for j in range(targets_num):
                tar2 = targetpos[j,:]
                self.distance_mat[i,j] = np.linalg.norm(tar1 - tar2, ord=2)
                if i == j:
                    self.distance_mat[i,j] = np.Inf
        self.logger = logger
        self.distance_mat_nochange = self.distance_mat.copy()
        self.unvisited_num = targets_num
        self.current_targets = np.zeros(drone_num, dtype=int)
        self.min_dist_step = [] #for debuging algorithm performance
        self.unvisited = np.ones(targets_num, dtype=bool)
        self.threshold_factor = params.threshold_factor
        self.safety_distance_allocation = params.safety_distance_allocation
        self.uri_state_mat = params.uri_state_mat
        self.an = an

        # History
        self.history =[]
        for i in range(drone_num):
            self.history.append(np.empty((0,3)))


    def get_state_matrix(self, drone_num, init_flag=False):
        if init_flag:
            return np.load(str(os.getcwd())+ self.uri_state_mat +'/state_mat/state_mat_d'+str(drone_num)+'_k'+str(self.k)+'.npy')
        else:   
            if self.k > 1:
                self.k -= 1
            self.logger.log(f'k updated : {self.k}')
            # print('k updated:' , self.k)
            if self.k >=2:
                return np.load(str(os.getcwd())+ self.uri_state_mat +'/state_mat/state_mat_d'+str(drone_num)+'_k'+str(self.k)+'.npy')
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

    def update_history(self, target_idx, drone_idx, targetpos):
        self.history[drone_idx] = np.vstack((self.history[drone_idx], targetpos[target_idx,:]))
    
    def update_distance_mat(self, target_idx):
        self.distance_mat[:, target_idx] = np.inf

    def  update_kmeans_drone_num(self, drone_num, targetpos, targetpos_reallocate):
        self.logger.log('kmeans updated')
        first_itr = 1
        self.min_dist_candidate = self.safety_distance_allocation
        drone_num_changed = 0
        while (self.min_dist_candidate < self.safety_distance_allocation or first_itr == 1) and (drone_num > 1):
            first_itr = 0
            #get initial target by Kmeans
            self.kmeans = KMeans(n_clusters=drone_num).fit(targetpos[self.unvisited,:])
            self.centers = self.kmeans.cluster_centers_
            self.current_targets = np.zeros(drone_num, dtype=int)
            unvisited_targets_temp = targetpos_reallocate.copy()
            for i in range(drone_num): 
                self.current_targets[i] = self.get_1_min(unvisited_targets_temp, self.centers[i,:])
                unvisited_targets_temp[self.current_targets[i],:] = np.inf 
            
            # calc initial dist vec
            self.combs = list(combinations(list(range(drone_num)), 2))
            self.combs_size = len(self.combs)
            self.initial_dist_vec = np.zeros(self.combs_size)
            for k in range(self.combs_size):
                i,j = self.combs[k]
                self.initial_dist_vec[k] = self.distance_mat[self.current_targets[i], self.current_targets[j]]  

            self.min_dist_candidate = min(self.initial_dist_vec)
            self.threshold_dist = self.threshold_factor * self.min_dist_candidate
            
            if (self.min_dist_candidate < self.safety_distance_allocation ) and (drone_num > 1) :
                drone_num -= 1
                drone_num_changed = 1
                self.logger.log(f'drones number updated: {drone_num}' )
        if drone_num > 1:
            self.logger.log(f'min_dist_org = {round(self.min_dist_candidate, 2)}')
            self.logger.log(f'threshold dist = {round(self.threshold_dist, 2)}')
            self.an.an_allocation(self.min_dist_candidate, drone_num , combination=self.current_targets ,threshold=self.threshold_dist)
        return drone_num, drone_num_changed

    def get_knn_last_drone(self):
        temp_distance_matrix = self.distance_mat.copy()
        # set columns of current targets to inf so they will not be selected again
        temp_distance_matrix[:, self.current_targets[0]] = np.inf
        # find nearest target
        return np.argsort(temp_distance_matrix[self.current_targets[0], :])[0]
          


class Allocation:
    def __init__(self, logger, an):
        self.logger = logger
        self.drone_num = params.drone_num
        self.drone_num_changed = 0
        self.targetpos = params.targetpos
        self.targets_num = params.targets_num
        self.targetpos_reallocate = self.targetpos.copy()
        self.an = an
        self.optim = Optim(self.targets_num, self.targetpos, self.logger, self.an)
        if self.drone_num > 1:
            self.state_mat = self.optim.get_state_matrix(self.drone_num, init_flag=True)
        self.drone_num, self.drone_num_changed = self.optim.update_kmeans_drone_num(self.drone_num, self.targetpos, self.targetpos_reallocate) # update drone_num for safety distance
        self.base = params.base
        if self.drone_num > 1:
            self.optimal_drone2target()
        
    def optimal_drone2target(self, dm=None):
        self.logger.log('calculate optimal drone2agent')
        options = list(permutations(range(self.drone_num))) 
        distance_mat = np.zeros([self.drone_num, self.drone_num])
        for i in range(self.drone_num):  # i = target pos
            tar1 = self.targetpos[self.optim.current_targets[i],:]
            for j in range(self.drone_num): # j = drone base
                if dm != None:
                    tar2 = np.array(dm.drones[j].start_coords)
                else:
                    tar2 = np.array(self.base[j])
                distance_mat[i,j] = np.linalg.norm(tar1 - tar2, ord=2)
        min_dist = np.inf
        for comb in options:
            comb = list(comb)
            current_dist = 0
            j=0
            for idx in comb:
                current_dist += distance_mat[idx, j]
                j+=1
            if current_dist < min_dist:
                min_dist = current_dist
                best_comb = comb
        self.optim.current_targets = self.optim.current_targets[best_comb]

    def update_kmeans(self, dm=None):
        self.logger.log('-------kmeans mode-------')
        while (self.optim.unvisited_num < self.drone_num) and (self.drone_num > 1):
            self.drone_num -= 1
        self.logger.log(f'drone number updated: {self.drone_num}')
        if self.drone_num > 1:
            self.drone_num, self.drone_num_changed = self.optim.update_kmeans_drone_num(self.drone_num, self.targetpos, self.targetpos_reallocate) 
            if self.drone_num > 1:
                self.optimal_drone2target(dm)   
        if self.drone_num == 1:
            self.optim.current_targets = np.zeros(1, dtype=int) 
            self.optim.current_targets[0] = self.optim.get_knn_last_drone()  
            self.an.an_allocation( 0, self.drone_num , self.optim.current_targets ,self.optim.threshold_dist) 

    def allocate(self, allocate_to):
        if self.drone_num > 1:
            self.optim.min_dist_step.append(self.optim.min_dist_candidate)
    
        if (self.optim.unvisited_num >= 2 * self.drone_num) and (self.drone_num > 1):
            #check k value a(avoid choose inf in get_knn function)
            if (self.optim.unvisited_num - self.drone_num < self.optim.k * self.drone_num and self.optim.k >= 2) or (self.drone_num_changed == 1):
                self.state_mat = self.optim.get_state_matrix(self.drone_num)
                self.drone_num_changed = 0

            # FIND KNN
            knn = self.optim.get_knn(allocate_to, self.drone_num)
            
            # search best combination 
            best_comb_candidate, self.optim.min_dist_candidate = self.optim.search_best_combination( self.drone_num, self.state_mat, knn)
    
            # check if best solution satisfy threshold
            if self.optim.min_dist_candidate > self.optim.threshold_dist:
                self.optim.current_targets = best_comb_candidate
                self.an.an_allocation( self.optim.min_dist_candidate, self.drone_num , self.optim.current_targets ,self.optim.threshold_dist)
            else:
                return 'update_kmeans'
        
        elif (self.optim.unvisited_num < 2 * self.drone_num) and (self.drone_num > 1):
            return 'update_kmeans'

        if (self.drone_num == 1) and (self.optim.unvisited_num > 0):
            # assign the nearest target as new target
            self.optim.current_targets[0] = self.optim.get_knn_last_drone()
            self.an.an_allocation( 0, self.drone_num , self.optim.current_targets ,self.optim.threshold_dist)



