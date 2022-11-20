#! /usr/bin/env python
import numpy as np
from Allocation_algorithm_init import Drone , Optim
from itertools import permutations 


class Allocation:
    def __init__(self, drone_num, targets, safety_distance, k_init, magazine):
        self.drone = Drone(drone_num=drone_num, safety_distance = safety_distance, magazine=magazine)
        self.targets = targets
        self.optim = Optim(self.targets, self.drone, k = k_init)
        if drone_num > 1:
            self.state_mat = self.optim.get_state_matrix(self.drone.drone_num, init_flag=True)
        self.optim.update_kmeans_drone_num(self.drone, self.targets) # update drone_num for safety distance
        if self.drone.drone_num > 1:
            self.optimal_drone2target()

    
    def optimal_drone2target(self):
        print('calc optimal drone2agent')
        options = list(permutations(range(self.drone.drone_num))) 
        distance_mat = np.zeros([self.drone.drone_num, self.drone.drone_num])
        for i in range(self.drone.drone_num):  # i = target pos
            tar1 = self.targets.targetpos[self.optim.current_targets[i],:]
            for j in range(self.drone.drone_num): # j = drone base
                tar2 = np.array(self.drone.base[j])
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
    

    def update_kmeans(self):
        while (self.optim.unvisited_num < self.drone.drone_num) and (self.drone.drone_num > 1):
            self.drone.drone_num -= 1
        print('drone number updated:', self.drone.drone_num)
        if self.drone.drone_num > 1:
            self.optim.update_kmeans_drone_num(self.drone, self.targets)
            if self.drone.drone_num > 1:
                self.optimal_drone2target()   
        if self.drone.drone_num == 1:
            self.optim.current_targets = np.zeros(1, dtype=int) ################################# add
            self.optim.current_targets[0] = self.optim.get_knn_last_drone()   


    def allocate(self, allocation_availability):
        if self.drone.drone_num > 1:
            self.optim.min_dist_step.append(self.optim.min_dist_candidate)
    
        if (self.optim.unvisited_num >= 2 * self.drone.drone_num) and (self.drone.drone_num > 1):

            #check k value a(avoid choose inf in get_knn function)
            if (self.optim.unvisited_num - self.drone.drone_num < self.optim.k * self.drone.drone_num and self.optim.k >= 2) or (self.drone.drone_num_changed == 1):
                self.state_mat = self.optim.get_state_matrix(self.drone.drone_num)
                self.drone.drone_num_changed = 0

            # FIND KNN
            knn = self.optim.get_knn(allocation_availability, self.drone.drone_num)

            # search best combination 
            best_comb_candidate, self.optim.min_dist_candidate = self.optim.search_best_combination( self.drone.drone_num, self.state_mat, knn)
    
            # check if best solution satisfy threshold
            if self.optim.min_dist_candidate > self.optim.threshold_dist:
                self.optim.current_targets = best_comb_candidate
            else:
                return 'update_kmeans'
        
        elif (self.optim.unvisited_num < 2 * self.drone.drone_num) and (self.drone.drone_num > 1):
            return 'update_kmeans'

        if (self.drone.drone_num == 1) and (self.optim.unvisited_num > 0):
            # assign the nearest target as new target
            self.optim.current_targets[0] = self.optim.get_knn_last_drone()


