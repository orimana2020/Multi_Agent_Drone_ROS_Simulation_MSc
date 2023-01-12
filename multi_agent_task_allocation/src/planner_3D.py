#! /usr/bin/env python

import numpy as np
from scipy import interpolate
import params
import Additionals



class Trajectory(object):
    def __init__(self, drones, logger):
        self.drone_num = len(drones)
        self.logger = logger
        self.res = params.resolution
        self.break_trajectory_len_factor = params.break_trajectory_len_factor
        self.minimum_floor_distance = params.floor_safety_distance # [m]
        self.retreat_dist = params.retreat_range
        x_span, y_span, z_span = params.span
        self.grid_3d = np.zeros([round(z_span/self.res), round(y_span/self.res), round(x_span/self.res)], dtype=int) #z y x
        minimum_floor_idx = round(self.minimum_floor_distance / self.res) + 1 # minimum floor 
        self.grid_3d[:minimum_floor_idx] = 1
        self.grid_3d_initial = self.grid_3d.copy()
        self.grid_3d_shape = self.grid_3d.shape
        print(f'3d grid shape: {self.grid_3d_shape}')
        self.visited_3d = np.zeros([round(z_span/self.res), round(y_span/self.res), round(x_span/self.res)], dtype=int) #z y x
        _,_,self.y_offset,_,_,_ = params.limits_idx
        z_lim, y_lim, x_lim = self.grid_3d.shape
        self.x_lim = x_lim -1
        self.y_lim = y_lim -1
        self.z_lim = z_lim -1 
        self.safety_distance = params.safety_distance_trajectory
        self.block_volume = [[]] * self.drone_num
        self.block_volumes_m = [[]] * self.drone_num # used for visualization only
        self.paths_m = [[]] * self.drone_num # used for visualization only
        self.smooth_path_m =[[]] * self.drone_num
        self.path_gt = [[]] * self.drone_num
        self.constant_blocking_area = [[]] * self.drone_num
        self.constant_blocking_area_m = [[]] * self.drone_num
        self.mean_x_targets_position = params.mean_x_targets_position
        self.smooth_points_num = params.points_in_smooth_params
        # self.error_arr = Additionals.generate_fake_error_mapping()
        self.error_arr = params.LPS_n_safety_vol #[x,y,z], resolution =0.05 # for inflate
        self.error_arr_max = np.max(self.error_arr) # for inflate
        self.dw_dist_idx = np.int8(np.round(params.downwash_distance / self.res)) 
        self.downwash_aware = params.DOWNWASH_AWARE
        self.simulate_lps_error = params.SIMULATE_LPS_ERROR # add error to path planning 
        self.LPS_pos_error = params.LPS_positioning_error_m # add error to path planning 
        self.max_LPS_pos_error = np.max(self.LPS_pos_error) # add error to path planning 
        # self.targetpos_max_x_diff = params.targetpos_max_x_diff
        floor = []  # generate floor block volume to visualize
        z_floor, y_floor, x_floor = self.grid_3d_initial[:minimum_floor_idx].shape
        for z in range(z_floor):
            for y in range(y_floor):
                for x in range(x_floor):
                    floor.append([z,y,x])
        self.block_volume_floor_m = self.convert_idx2meter(np.array(floor))
        
        for j in range(self.drone_num):
            start = self.covert_meter2idx(drones[j].base)
            mean_intermidiate = self.covert_meter2idx(np.array(drones[j].base) + np.array([self.mean_x_targets_position * self.break_trajectory_len_factor, 0,0]) )
            path = [start]
            next = np.array(start) + np.array([0,0,1],dtype=int)
            while not (next == mean_intermidiate).all():
                path.append((next[0],next[1],next[2]))
                next += np.array([0,0,1],dtype=int)
            path.append(mean_intermidiate)
            self.constant_blocking_area[j] = self.inflate(path, goal_title=None)
            self.constant_blocking_area_m[j] = self.convert_idx2meter(self.constant_blocking_area[j])

        
    def get_neighbors(self, current):
        neighbors = []
        dist = []
        # (Z,Y,X) current[0] = z, current[1] = y, current[2] = x
        # Down Y-
        if current[1] > 0 and self.visited_3d[current[0],current[1]-1, current[2]] == 0:
            neighbors.append((current[0],current[1]-1, current[2]))
            dist.append(1)

        # UP Y+
        if current[1] < self.y_lim and self.visited_3d[current[0],current[1]+1, current[2]] == 0:
            neighbors.append((current[0],current[1]+1, current[2]))
            dist.append(1)
        
        # RIGHT X+
        if current[2] < self.x_lim and self.visited_3d[current[0], current[1],current[2]+1] == 0:
            neighbors.append((current[0], current[1],current[2]+1))
            dist.append(1)
        
        # LEFT X-
        if current[2] > 0 and self.visited_3d[current[0], current[1],current[2]-1] == 0:
            neighbors.append((current[0], current[1],current[2]-1))
            dist.append(1)
        
        # IN Z+
        if current[0] < self.z_lim and self.visited_3d[current[0]+1, current[1],current[2]] == 0:
            neighbors.append((current[0]+1, current[1],current[2]))
            dist.append(1)

        # out Z-
        if current[0] > 0 and self.visited_3d[current[0]-1, current[1],current[2]] == 0:
            neighbors.append((current[0]-1, current[1],current[2]))
            dist.append(1)
        
        # Down Y- RIGHT X+
        if current[1] > 0 and current[2] < self.x_lim and self.visited_3d[current[0], current[1]-1 ,current[2]+1] == 0:
            neighbors.append((current[0], current[1]-1 ,current[2]+1))
            dist.append(1.414)

        # Down Y- LEFT X-
        if current[1] > 0 and current[2] > 0 and self.visited_3d[current[0], current[1]-1 ,current[2]-1] == 0:
            neighbors.append((current[0], current[1]-1 ,current[2]-1))
            dist.append(1.414)

        #  Down Y- IN Z+ 
        if current[1] > 0 and current[0] < self.z_lim and self.visited_3d[current[0]+1, current[1]-1 ,current[2]] == 0:
            neighbors.append((current[0]+1, current[1]-1 ,current[2]))
            dist.append(1.414)
        
        #  Down Y- out Z-
        if current[1] > 0 and current[0] > 0 and self.visited_3d[current[0]-1, current[1]-1 ,current[2]] == 0:
            neighbors.append((current[0]-1, current[1]-1 ,current[2])) 
            dist.append(1.414)   

        # UP Y+ RIGHT X+
        if current[1] < self.y_lim and current[2] < self.x_lim and self.visited_3d[current[0], current[1]+1 ,current[2]+1] == 0:
            neighbors.append((current[0], current[1]+1 ,current[2]+1)) 
            dist.append(1.414)

        # UP Y+ LEFT X-
        if current[1] < self.y_lim and current[2] > 0 and self.visited_3d[current[0], current[1]+1 ,current[2]-1] == 0:
            neighbors.append((current[0], current[1]+1 ,current[2]-1)) 
            dist.append(1.414)    

        #  UP Y+ IN Z+ 
        if current[1] < self.y_lim and current[0] < self.z_lim and self.visited_3d[current[0]+1, current[1]+1 ,current[2]] == 0:
            neighbors.append((current[0]+1, current[1]+1 ,current[2]))   
            dist.append(1.414)

        #  UP Y+ out Z-
        if current[1] < self.y_lim and current[0] > 0 and self.visited_3d[current[0]-1, current[1]+1 ,current[2]] == 0:
            neighbors.append((current[0]-1, current[1]+1 ,current[2]))  
            dist.append(1.414) 

        # UP Y+ out Z-  RIGHT X+
        if current[0] > 0 and current[1] < self.y_lim and current[2] < self.x_lim and self.visited_3d[current[0]-1, current[1]+1 ,current[2]+1] == 0:
            neighbors.append((current[0]-1, current[1]+1 ,current[2]+1))   
            dist.append(1.732) 

        # UP Y+ out Z-  LEFT X-
        if current[0] > 0 and current[1] < self.y_lim and current[2] > 0 and self.visited_3d[current[0]-1, current[1]+1 ,current[2]-1] == 0:
            neighbors.append((current[0]-1, current[1]+1 ,current[2]-1)) 
            dist.append(1.732)       

        # Down Y- out Z-  RIGHT X+
        if current[0] > 0 and current[1] > 0 and current[2] < self.x_lim and self.visited_3d[current[0]-1, current[1]-1 ,current[2]+1] == 0:
            neighbors.append((current[0]-1, current[1]-1 ,current[2]+1)) 
            dist.append(1.732) 

        #  Down Y- out Z-  LEFT X-
        if current[0] > 0 and current[1] > 0 and current[2] > 0 and self.visited_3d[current[0]-1, current[1]-1 ,current[2]-1] == 0:
            neighbors.append((current[0]-1, current[1]-1 ,current[2]-1))   
            dist.append(1.732)   

        # UP Y+ IN Z+  RIGHT X+
        if current[0] < self.z_lim and current[1] < self.y_lim and current[2] < self.x_lim and self.visited_3d[current[0]+1, current[1]+1 ,current[2]+1] == 0:
            neighbors.append((current[0]+1, current[1]+1 ,current[2]+1)) 
            dist.append(1.732) 

        # UP Y+ IN Z+  LEFT X-
        if current[0] < self.z_lim and current[1] < self.y_lim and current[2] > 0 and self.visited_3d[current[0]+1, current[1]+1 ,current[2]-1] == 0:
            neighbors.append((current[0]+1, current[1]+1 ,current[2]-1)) 
            dist.append(1.732) 

        # Down Y- IN Z+  RIGHT X+
        if current[0] < self.z_lim and current[1] > 0 and current[2] < self.x_lim and self.visited_3d[current[0]+1, current[1]-1 ,current[2]+1] == 0:
            neighbors.append((current[0]+1, current[1]-1 ,current[2]+1)) 
            dist.append(1.732) 

        #  Down Y- IN Z+  LEFT X-
        if current[0] < self.z_lim and current[1] > 0 and current[2] > 0 and self.visited_3d[current[0]+1, current[1]-1 ,current[2]-1] == 0:
            neighbors.append((current[0]+1, current[1]-1 ,current[2]-1))
            dist.append(1.732) 

        return neighbors, dist

    def h(self, p1, p2):
        z1, y1, x1  = p1
        z2, y2, x2 = p2
        return np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)


    def reconstruct_path(self, came_from, current, start):
        path = []
        while current in came_from.keys():
            path.append(current)
            current = came_from[current]
        path.append(start)
        return np.array(path[::-1])


    def A_star(self, start, goal):
        came_from = {}
        open_set = {}
        g_score = np.ones(self.grid_3d.shape) * np.inf
        g_score[start] = 0
        f_score = np.ones(self.grid_3d.shape) * np.inf
        f_score[start] = self.h(start, goal)
        open_set[start] = f_score[start]

        while len(open_set) != 0:
            current = min(open_set, key=open_set.get)
            if current == goal:
                return self.reconstruct_path(came_from, current, start)
            
            del open_set[current]
            self.visited_3d[current] = 1 # 1 mark as visited
            neighbors, dist = self.get_neighbors(current)
            for i in range(len(dist)):
                neigbor = neighbors[i]
                tentative_g_score = g_score[current] + dist[i]
                if tentative_g_score < g_score[neigbor]:
                    came_from[neigbor] = current
                    g_score[neigbor] = tentative_g_score
                    f_score[neigbor] = tentative_g_score + self.h(neigbor, goal)
                    if neigbor not in open_set.keys():
                        open_set[neigbor] = f_score[neigbor]
        return False #could not find path


    def get_smooth_path(self, path ,len1 ,len3):
        # s = smoothness, m > k must hold, default k degree is  k=3, m is number of points
        try:
            weights = np.ones(len(path))*10
            weights[0:len1] = 100
            weights[len(path)-len3:] = 100
            tck, _ = interpolate.splprep([path[:,0], path[:,1], path[:,2]],w=weights,s=10)  
            u_fine = np.linspace(0,1,self.smooth_points_num) # determine number of points in smooth path 
            smooth_path = interpolate.splev(u_fine, tck)
            return np.transpose(np.array(smooth_path))
        except: # remove duplicated coordes which result error in interpolation
            new_path = [path[0]]
            for i in range(len(path)):
                a = np.round(np.array(path[i]) / self.res)
                b = np.round(np.array(new_path[-1]) / self.res)
                if not (a == b).all():
                    new_path.append(path[i])
            path2 = np.array(new_path)
            weights = np.ones(len(path2))*10
            weights[0:len1] = 100
            weights[len(path2)-len3:] = 100
            tck, _ = interpolate.splprep([path2[:,0], path2[:,1], path2[:,2]],w=weights,s=10)  
            u_fine = np.linspace(0,1,self.smooth_points_num) # determine number of points in smooth path 
            smooth_path = interpolate.splev(u_fine, tck)
            self.logger.log('duplicate coords found in path and resolved')
            return np.transpose(np.array(smooth_path))


    def inflate(self, path, goal_title): # adaptive circle
        block_volume = []
        for node in path:
            z0, y0, x0 = node
            try:
                distance_idx = self.error_arr[x0, y0, z0]
            except:
                distance_idx = self.error_arr_max
            dist_power2 = distance_idx**2
            for z in range(-distance_idx, distance_idx+1,1):
                for y in range(-distance_idx, distance_idx+1,1):
                    if ((y)**2 + (z)**2) < dist_power2:
                        if not z+z0 > self.z_lim - 1 and not y+y0 > self.y_lim - 1 and not y+y0 < 0 and not z+z0 < 0:
                            block_volume.append((z+z0,y+y0,x0))
        
        # add downwash safety distance around the target
        if self.downwash_aware:
            if goal_title == 'target':
                goal_z, goal_y, goal_x = path[-1,:]
                for z in range(goal_z - self.dw_dist_idx[2][0], goal_z + self.dw_dist_idx[2][0] + 1):
                    if 0 <= z < self.grid_3d_shape[0]:
                        for y in range(goal_y - self.dw_dist_idx[1][0], goal_y + self.dw_dist_idx[1][0] + 1):
                            if 0 <= y < self.grid_3d_shape[1]:
                                # for x in range(goal_x - self.dw_dist_idx[0][0], goal_x + self.dw_dist_idx[0][1] + 1):
                                for x in range(self.dw_dist_idx[0][0],  self.dw_dist_idx[0][1] + 1):
                                    if 0 <= x < self.grid_3d_shape[2]:
                                        block_volume.append((z,y,x))
        return np.array(block_volume)


    def covert_meter2idx(self, coords_meter): # (x,y,z) -> (z,y,x)
        return (round(coords_meter[2]/self.res ), round(coords_meter[1]/self.res + (-self.y_offset))  , round(coords_meter[0]/self.res ) ) 

    def convert_idx2meter(self, coords_idx): #(z,y,x) -> (x,y,z)       
        return np.stack(((coords_idx[:,2]) * self.res, (coords_idx[:,1] - (-self.y_offset)) * self.res, coords_idx[:,0] * self.res), axis=-1)        


    def get_path(self, start_m, goal_m, start_accurate_m, is_forward):
        """
        find path index, convert to [m], replace start and end to accurate values
        """
        if is_forward:
            if start_m[0] > goal_m[0]:
                temp = goal_m
                goal_m = start_m
                start_m = temp
            start = self.covert_meter2idx(start_m)
            goal = self.covert_meter2idx(goal_m)
            break_trajecoty_len = abs(start_m[0] - goal_m[0]) * self.break_trajectory_len_factor
            intermidiate_1 = self.covert_meter2idx((start_m[0] + break_trajecoty_len, start_m[1], start_m[2]))
            intermidiate_2 = self.covert_meter2idx((goal_m[0] - break_trajecoty_len, goal_m[1], goal_m[2]))
        else: #backward
            if start_m[0] < goal_m[0]:
                temp = goal_m
                goal_m = start_m
                start_m = temp
            start = self.covert_meter2idx(start_m)
            goal = self.covert_meter2idx(goal_m)
            break_trajecoty_len = abs(start_m[0] - goal_m[0]) * self.break_trajectory_len_factor
            intermidiate_1 = self.covert_meter2idx((start_m[0] - break_trajecoty_len, start_m[1], start_m[2]))
            intermidiate_2 = self.covert_meter2idx((goal_m[0] + break_trajecoty_len, goal_m[1], goal_m[2]))
        if self.grid_3d[goal] == 1 or self.grid_3d[intermidiate_1] == 1 or self.grid_3d[intermidiate_2] == 1: # fast sanity check of goal occupancy status
            self.logger.log('Sanity check failed')
            return None 
        
        try:
            path1 = self.A_star(start, intermidiate_1)
            path1 = path1[:-1]
            self.visited_3d = self.grid_3d.copy()
            path2 = self.A_star(intermidiate_1, intermidiate_2)
            path2 = path2[:-1]
            self.visited_3d = self.grid_3d.copy()
            path3 = self.A_star(intermidiate_2, goal)
            self.visited_3d = self.grid_3d.copy()
            path = np.vstack((path1, path2, path3))
            # -------- convert idx 2 meter
            segment1_m = self.convert_idx2meter(path1)
            segment2_m = self.convert_idx2meter(path2)
            segment3_m = self.convert_idx2meter(path3)
            # --------- replace to accurate position of start and goal
            segment1_m[0,0] = start_accurate_m[0] 
            segment1_m[:,1] ,segment1_m[:,2] =  start_accurate_m[1], start_accurate_m[2] #start y,z values
            segment3_m[:,1], segment3_m[:,2] = goal_m[1], goal_m[2] # goal y,z values
            segment3_m[-1,0] = goal_m[0]
            return segment1_m, segment2_m, segment3_m, path
        except:
            self.logger.log('error in creating path segments')
            return None


    def plan(self, drones ,drone_idx, drone_num):
        self.logger.log(f'start path planning for drone {drone_idx}')
        start_m = drones[drone_idx].start_coords
        start_accurate_m = drones[drone_idx].accurate_coords
        goal_m = drones[drone_idx].goal_coords
        start_title = drones[drone_idx].start_title
        goal_title = drones[drone_idx].goal_title
        self.grid_3d = self.grid_3d_initial.copy()
        for i in range(drone_num): # update grid_3D, exclude current drone block_volume
            if (i != drone_idx):
                self.grid_3d[self.constant_blocking_area[i][:,0], self.constant_blocking_area[i][:,1], self.constant_blocking_area[i][:,2]] = 1
                if (len(self.block_volume[i]) > 0) and not (drones[i].at_base):
                    self.grid_3d[self.block_volume[i][:,0], self.block_volume[i][:,1], self.block_volume[i][:,2]] = 1
        self.visited_3d = self.grid_3d.copy()

        if (start_title == 'base' and goal_title == 'target') or (start_title == 'target' and goal_title == 'base'):
            try:
                if (start_title == 'base' and goal_title == 'target'):
                    segment1_m, segment2_m, segment3_m, path = self.get_path(start_m, goal_m, start_accurate_m ,is_forward=True)
                elif (start_title == 'target' and goal_title == 'base'):
                    segment1_m, segment2_m, segment3_m, path = self.get_path(goal_m, start_m,start_accurate_m ,is_forward=False)
                self.block_volume[drone_idx] = self.inflate(path, goal_title)
                self.paths_m[drone_idx] = np.vstack((segment1_m, segment2_m, segment3_m))
                self.smooth_path_m[drone_idx] = self.get_smooth_path(path=self.paths_m[drone_idx],len1=len(segment1_m),len3=len(segment3_m))  
                self.block_volumes_m[drone_idx] = self.convert_idx2meter(self.block_volume[drone_idx])
                if self.simulate_lps_error:
                    self.path_gt[drone_idx] = self.smooth_path_m[drone_idx]
                    self.smooth_path_m[drone_idx] = self.get_path_LPS_error(self.smooth_path_m[drone_idx], drones, drone_idx)
                self.logger.log(f'path found drone {drone_idx}')
                return 1
            except:
                self.logger.log(f'No Path Found! drone {drone_idx} from {start_title} to {goal_title} , start: {np.round((np.array(start_m)),2)} , goal: {np.round((np.array(goal_m)),2)}')
                return 0

        elif (start_title == 'target' and goal_title == 'target'):
            intermidiate_m = (min(start_m[0], goal_m[0]) - self.retreat_dist, (start_m[1]+goal_m[1])/2, (start_m[2]+goal_m[2])/2)
            try:
                segment1_m, segment2_m, segment3_m, path = self.get_path(start_m, intermidiate_m, start_accurate_m ,is_forward=False)
                block_volume1 = self.inflate(path, goal_title=None) #because we don't want to add inflation at intermidiate point
                path1_m = np.vstack((segment1_m, segment2_m, segment3_m))
                smooth_path_m1 = self.get_smooth_path(path=path1_m, len1=len(segment1_m), len3=len(segment3_m))
                block_volume1_m = self.convert_idx2meter(block_volume1)
                    
                segment1_m, segment2_m, segment3_m, path = self.get_path(intermidiate_m, goal_m, intermidiate_m,is_forward=True)
                block_volume2 = self.inflate(path, goal_title)
                path2_m = np.vstack((segment1_m, segment2_m, segment3_m))
                smooth_path_m2 = self.get_smooth_path(path=path2_m, len1=len(segment1_m), len3=len(segment3_m))
                block_volume2_m = self.convert_idx2meter(block_volume2)

                self.block_volume[drone_idx] = np.vstack((block_volume1, block_volume2))
                self.paths_m[drone_idx] = np.vstack((path1_m, path2_m))
                self.smooth_path_m[drone_idx] = np.vstack((smooth_path_m1, smooth_path_m2))
                self.block_volumes_m[drone_idx] = np.vstack((block_volume1_m, block_volume2_m))
                if self.simulate_lps_error:
                    self.path_gt[drone_idx] = self.smooth_path_m[drone_idx]
                    self.smooth_path_m[drone_idx] = self.get_path_LPS_error(self.smooth_path_m[drone_idx], drones, drone_idx)
                return 1
            except:
                self.logger.log(f'No Path Found! drone {drone_idx} from {start_title} to {goal_title} , start: {np.round((np.array(start_m)),2)} , goal: {np.round((np.array(goal_m)),2)}')
                return 0

    def get_path_LPS_error(self, smooth_path_m, drones, drone_idx):
        path_fake_error = []
        signx = np.random.choice([-1,1])
        signy = np.random.choice([-1,1])
        signz = np.random.choice([-1,1])
        for coords in smooth_path_m:
            x,y,z = np.int8(coords / self.res)
            try:
                pos_error = self.LPS_pos_error[x, y, z] 
            except:
                pos_error = self.max_LPS_pos_error
            random_err = np.random.normal(pos_error, 0.015)
            
            new_coords = coords + np.array([signx * random_err/2**0.5, 
            signy * random_err/2**0.5,
            signz * np.random.normal(0.05, 0.005) ])
            path_fake_error.append(new_coords)
        drones[drone_idx].goal_coords = path_fake_error[-1]
        return np.array(path_fake_error)
            
            

        



# def inflate_squre(self, path):
    #     distance_idx = round(self.safety_distance/self.res)
    #     block_volume = []
    #     for node in path:
    #         for z in range(node[0]-3,node[0]+3):
    #             if z > self.z_lim - 1:
    #                 z = self.z_lim -1
    #             if z < 0:
    #                 z = 0
    #             for y in range(node[1]-distance_idx,node[1]+distance_idx):
    #                 if y > self.y_lim - 1:
    #                     y = self.y_lim -1
    #                 if y < 0:
    #                     y = 0
    #                 for x in range(node[2]-distance_idx, node[2]+ distance_idx):
    #                     if x > self.x_lim - 1:
    #                         x = self.x_lim -1
    #                     if x < 0:
    #                         x = 0
    #                     block_volume.append((z,y,x))
    #     return np.array(block_volume)


    # def inflate_circle(self, path):
    #     distance_idx = round(self.safety_distance/self.res)
    #     dist_power2 = distance_idx**2
    #     block_volume = []
    #     for node in path:
    #         z0, y0, x0 = node
    #         for z in range(-distance_idx, distance_idx+1,1):
    #             for y in range(-distance_idx, distance_idx+1,1):
    #                 if ((y)**2 + (z)**2) <  dist_power2:
    #                     if not z+z0 > self.z_lim - 1 and not y+y0 > self.y_lim - 1 and not y+y0 < 0 and not z+z0 < 0:
    #                         block_volume.append((z+z0,y+y0,x0))
    #     return np.array(block_volume)