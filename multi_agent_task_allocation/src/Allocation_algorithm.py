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
            tar1 = targetpos[i,1:] # ------------- change to yz dist change to yz only!
            for j in range(targets_num):
                tar2 = targetpos[j,1:] # ------------- change to yz dist change to yz only!
                self.distance_mat[i,j] = np.linalg.norm(tar1 - tar2, ord=2)
                if i == j:
                    self.distance_mat[i,j] = np.Inf
        self.targetpos = targetpos
        self.logger = logger
        self.distance_mat_nochange = self.distance_mat.copy()
        self.unvisited_num = targets_num
        self.current_targets = np.zeros(drone_num, dtype=int)
        self.unvisited = np.ones(targets_num, dtype=bool)
        self.threshold_factor = params.threshold_factor
        self.threshold_dist = None
        self.downwash_distance = params.downwash_distance
        self.uri_state_mat = params.uri_state_mat
        self.an = an
        self.a0 = -self.downwash_distance[2][0]/ self.downwash_distance[1][0] 
        self.b0 = self.downwash_distance[2][0] 
        self.cost = np.inf
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
            print('k updated:' , self.k)
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
        min_cost = np.inf
        for i in range(self.k ** drone_num):
            next_targets = np.sum(knn * state_mat[:,:,i], axis=0)
            dist_vec = np.zeros(self.combs_size)
            for j in range(self.combs_size):
                dist_vec[j] = self.distance_mat_nochange[next_targets[self.combs[j][0]], next_targets[self.combs[j][1]]]
            # change in distancenes between initial distance and current distance    
            next_diff = np.linalg.norm(dist_vec - self.initial_dist_vec, ord=2) 
            # traveled distance between current targets and next targets
            travel_dist_vec = np.zeros(drone_num)
            for j in range(drone_num):
                if self.current_targets[j] != next_targets[j]:
                    travel_dist_vec[j] = self.distance_mat_nochange[self.current_targets[j], next_targets[j]]
            travel_dist = np.linalg.norm(travel_dist_vec, ord=2)
            # total cost
            current_cost = next_diff + travel_dist
            if self.is_in_dw_volume(next_targets):
                current_cost *= 5
            if current_cost < min_cost:
                min_cost = current_cost
                min_dist = min(dist_vec)
                best_comb = next_targets
        return best_comb, min_dist  


    def  update_kmeans(self, drone_num ,targetpos, targetpos_reallocate):
        self.logger.log('kmeans updated')
        self.kmeans = KMeans(n_clusters=drone_num).fit(targetpos[self.unvisited,1:]) # ------------- change to yz dist
        self.centers = self.kmeans.cluster_centers_
        self.current_targets = np.zeros(drone_num, dtype=int)
        unvisited_targets_temp = targetpos_reallocate.copy()
        for i in range(drone_num): 
            self.current_targets[i] = self.get_1_min(unvisited_targets_temp, self.centers[i,:]) 
            unvisited_targets_temp[self.current_targets[i],:] = np.inf 
        self.combs = list(combinations(list(range(drone_num)), 2))
        self.combs_size = len(self.combs)
    
    def get_initial_dist(self, drone_num):
        self.initial_dist_vec = np.zeros(self.combs_size)
        for k in range(self.combs_size):
            i,j = self.combs[k]
            self.initial_dist_vec[k] = self.distance_mat[self.current_targets[i], self.current_targets[j]]  
        self.threshold_dist = min(self.initial_dist_vec) * self.threshold_factor
        self.an.an_allocation(min(self.initial_dist_vec), drone_num, self.current_targets ,self.threshold_dist, is_kmeans=1)


    def get_1_min(self, targets, test_pnt): 
        diff = np.linalg.norm(targets[:,1:] - test_pnt, ord=2, axis=1) # ------------- change to yz dist   
        return np.argmin(diff)  

    def update_history(self, target_idx, drone_idx, targetpos):
        self.history[drone_idx] = np.vstack((self.history[drone_idx], targetpos[target_idx,:]))
    
    def update_distance_mat(self, target_idx):
        self.distance_mat[:, target_idx] = np.inf
    
    def is_in_dw_volume(self, next_targets):
        for comb in self.combs:
            y1,z1 = self.targetpos[next_targets[comb[0]],1], self.targetpos[next_targets[comb[0]],2]
            y2,z2 = self.targetpos[next_targets[comb[1]],1], self.targetpos[next_targets[comb[1]],2]
            dy = abs(y1-y2)
            dz = abs(z1-z2)
            if dy <= self.downwash_distance[1][0] and dz <= self.downwash_distance[2][0]:
                z = self.a0*dy + self.b0 
                if dz <= z: #inside -> not safe
                    return True
        return False

    def get_knn_last_drone(self):
        temp_distance_matrix = self.distance_mat.copy()
        temp_distance_matrix[:, self.current_targets[0]] = np.inf
        return np.argsort(temp_distance_matrix[self.current_targets[0], :])[0]
          
class Allocation:
    def __init__(self, logger, an):
        self.logger = logger
        self.drone_num = params.drone_num
        self.drone_num_changed = False
        self.targetpos = params.targetpos
        self.targets_num = params.targets_num
        self.targetpos_reallocate = self.targetpos.copy()
        self.base = params.base
        self.an = an
        self.optim = Optim(self.targets_num, self.targetpos, self.logger, self.an)
        if self.drone_num > 1:
            self.state_mat = self.optim.get_state_matrix(self.drone_num, init_flag=True)
            # self.optim.update_kmeans(self.drone_num, self.targetpos, self.targetpos_reallocate) # update drone_num for safety distance
            # self.optimal_drone2target()
            # self.optim.get_initial_dist(self.drone_num)
            

           
    def optimal_drone2target(self, dm):
        drones_idx = [i for i in range(self.drone_num)]
        targets_idx = self.optim.current_targets
        targets_y_coords = [self.targetpos[i,1] for i in self.optim.current_targets]
        drones_y_coords = [dm.drones[i].start_coords[1] for i in range(self.drone_num)]
        #sort targets
        sorted_targets_indices = np.argsort(targets_y_coords) 
        targets_idx[:] = [targets_idx[i] for i in sorted_targets_indices]

        # sort drones
        indices_y_drones = np.argsort(drones_y_coords)
        drones_idx[:] = [drones_idx[i] for i in indices_y_drones]

        # sort by drone idx
        indeces_drone_idx = np.argsort(drones_idx)
        drones_idx[:] = [drones_idx[i] for i in indeces_drone_idx]
        targets_idx[:] = [targets_idx[i] for i in indeces_drone_idx]
        self.optim.current_targets = np.array(targets_idx)


    def update_kmeans(self, dm):
        self.logger.log('-------kmeans mode-------')
        while (self.optim.unvisited_num < self.drone_num) and (self.drone_num > 1):
            self.drone_num -= 1
        self.logger.log(f'drone number updated: {self.drone_num}')
        if self.drone_num > 1:
            self.optim.update_kmeans(self.drone_num, self.targetpos, self.targetpos_reallocate) 
            self.optimal_drone2target(dm)  
            self.optim.get_initial_dist(self.drone_num)

        if self.drone_num == 1:
            self.optim.current_targets = np.zeros(1, dtype=int) 
            self.optim.current_targets[0] = self.optim.get_knn_last_drone()  
            self.an.an_allocation( 0, self.drone_num , self.optim.current_targets ,self.optim.threshold_dist,0) 

    def allocate(self, allocate_to):

        if (self.optim.unvisited_num < 2 * self.drone_num) and (self.drone_num > 1):
            self.drone_num_changed = True
            return 'remove_drone'

        elif (self.drone_num > 1):
            if (self.optim.unvisited_num - self.drone_num < self.optim.k * self.drone_num and self.optim.k >= 2) or self.drone_num_changed:
                self.drone_num_changed = False
                self.state_mat = self.optim.get_state_matrix(self.drone_num)

            knn = self.optim.get_knn(allocate_to, self.drone_num)
            best_comb_candidate, min_dist = self.optim.search_best_combination( self.drone_num, self.state_mat, knn)
            if  min_dist > self.optim.threshold_dist:
                self.optim.current_targets = best_comb_candidate
                self.an.an_allocation(min_dist, self.drone_num , self.optim.current_targets , self.optim.threshold_dist ,0)
            else:
                return 'update_kmeans'

        elif (self.drone_num == 1):
            self.optim.current_targets[0] = self.optim.get_knn_last_drone()
            self.an.an_allocation(0, self.drone_num , self.optim.current_targets , 0 ,0)

