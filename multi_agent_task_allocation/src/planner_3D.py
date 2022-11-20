#! /usr/bin/env python

import numpy as np
from scipy import interpolate


class Trajectory(object):
    def __init__(self, x_span ,y_span ,z_span ,drone_num ,res , safety_distance):
        self.res = res
        self.minimum_floor_distance = 0.3 # meter
        z_span = z_span - self.minimum_floor_distance
        self.grid_3d = np.zeros([int(z_span/res), int(y_span/res), int(x_span/res)], dtype=int) #z y x
        self.grid_3d_shape = self.grid_3d.shape
        self.visited_3d = np.zeros([int(z_span/res), int(y_span/res), int(x_span/res)], dtype=int) #z y x
        z_lim, y_lim, x_lim = self.grid_3d.shape
        self.x_lim = x_lim -1
        self.y_lim = y_lim -1
        self.z_lim = z_lim -1 
        self.safety_distance = safety_distance
        self.block_volume = []
        self.block_volumes_m = [] # used for visualization only
        self.paths_m = [] # used for visualization only
        self.smooth_path_m =[]
        
        for _ in range(drone_num):
            self.block_volume.append([])
            self.block_volumes_m.append([])
            self.paths_m.append([])
            self.smooth_path_m.append([])

        
    def get_neighbors(self, current):
        neighbors = []
        dist = []
        # (Z,Y,X)
        # current[0] = z
        # current[1] = y
        # current[2] = x

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
        # reconstruct_path(self, came_from:dict, current, start): for python 3
        path = []
        while current in came_from.keys():
            path.append(current)
            current = came_from[current]
        path.append(start)
        return np.array(path[::-1])



    def A_star(self,start,goal):
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


    def get_smooth_path(self, path):
        # s = smoothness, m > k must hold, default k degree is  k=3, m is number of points
        tck, _ = interpolate.splprep([path[:,0], path[:,1], path[:,2]], s=10)  
        # x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
        u_fine = np.linspace(0,1,int(len(path))) # determine number of points in smooth path 
        smooth_path = interpolate.splev(u_fine, tck)
        return np.transpose(np.array(smooth_path))

    def inflate(self, path):
        distance_idx = int(self.safety_distance/self.res)
        block_volume = []
        for node in path:
            for z in range(node[0]-3,node[0]+3):
                if z > self.z_lim - 1:
                    z = self.z_lim -1
                if z < 0:
                    z = 0
                for y in range(node[1]-distance_idx,node[1]+distance_idx):
                    if y > self.y_lim - 1:
                        y = self.y_lim -1
                    if y < 0:
                        y = 0
                    for x in range(node[2]-distance_idx, node[2]+ distance_idx):
                        if x > self.x_lim - 1:
                            x = self.x_lim -1
                        if x < 0:
                            x = 0

                        block_volume.append((z,y,x))
        return np.array(block_volume)


    def covert_meter2idx(self, coords_meter): # (x,y,z) -> (z,y,x)
        return (int((coords_meter[2]-self.minimum_floor_distance)/self.res ), int(coords_meter[1]/self.res + self.y_lim/2)  , int(coords_meter[0]/self.res ) ) 

    def convert_idx2meter(self, coords_idx, goal=None, add_end_point=0): #(z,y,x) -> (x,y,z)
        coord_m = np.stack(((coords_idx[:,2] ) * self.res, (coords_idx[:,1] - self.y_lim/2) * self.res, coords_idx[:,0] * self.res + self.minimum_floor_distance), axis=-1)
        if add_end_point == 0:
            return coord_m
        elif add_end_point == 1:
            # remove the last point 
            coord_m = coord_m[:-1]
            # add original target coords
            # return np.stack([coord_m, np.array(goal)],axis=0)
            return np.vstack((coord_m, np.array(goal)))
    
    def plan(self, start_m, goal_m, start_title, goal_title ,drone_idx, drone_num, at_base):
        
        # update grid_3D, exclude current drone block_volume
        self.grid_3d = np.zeros([self.grid_3d_shape[0], self.grid_3d_shape[1], self.grid_3d_shape[2]], dtype=int) #z y x - reset grid_3d
        for i in range(drone_num):
            if (i != drone_idx) and (len(self.block_volume[i]) > 0) and (at_base[i] == 0):
                self.grid_3d[self.block_volume[i][:,0], self.block_volume[i][:,1], self.block_volume[i][:,2]] = 1
        self.visited_3d = self.grid_3d.copy()

        # (start = base and goal = target) or (start = target and goal = base)
        if (start_title == 'base' and goal_title == 'target') or (start_title == 'target' and goal_title == 'base'):
            start = self.covert_meter2idx(start_m)
            goal = self.covert_meter2idx(goal_m)
            if self.grid_3d[goal] == 1: # fast sanity check of goal occupancy status
                print('sanity check failed')
                return 0 
            try:
                path = self.A_star(start, goal)
                smooth_path = self.get_smooth_path(path)
                self.block_volume[drone_idx] = self.inflate(path)      
                self.paths_m[drone_idx] = self.convert_idx2meter(path)      
                self.smooth_path_m[drone_idx] = self.convert_idx2meter(smooth_path,goal=goal_m,add_end_point=1)
                self.block_volumes_m[drone_idx] = self.convert_idx2meter(self.block_volume[drone_idx])
                print('Path Found')
                return 1
            except:
                print(' No Path Found! agent = '+str( drone_idx)+' from '+str(start_title)+ ' to '+ str(goal_title)+' start:'+str(start_m)+' goal:'+str(goal_m) + '')
                return 0

        elif (start_title == 'target' and goal_title == 'target'):
            retreat_dist = 0.5
            start = self.covert_meter2idx(start_m)
            # intermidiate = self.covert_meter2idx((start_m[0]-retreat_dist, start_m[1], start_m[2])) 
            intermidiate = self.covert_meter2idx((min([start_m[0],goal_m[0]])-retreat_dist, start_m[1], start_m[2])) 
            goal = self.covert_meter2idx(goal_m)
            if self.grid_3d[goal] == 1 or self.grid_3d[intermidiate] == 1: # fast sanity check of goal occupancy status
                print('sanity check failed')
                return 0 
            try:
                path1 = self.A_star(start, intermidiate)
                if len(path1)< 2:
                    print('intermidiate path not found!!!')
                smooth_path1 = self.get_smooth_path(path1)
                block_volume1 = self.inflate(path1) 
                self.visited_3d = self.grid_3d.copy()
                path2 = self.A_star(intermidiate, goal)
                smooth_path2 = self.get_smooth_path(path2)
                block_volume2 = self.inflate(path2) 
                path = np.vstack((path1, path2))
                smooth_path = np.vstack((smooth_path1, smooth_path2))
                self.block_volume[drone_idx] = np.vstack((block_volume1, block_volume2))  
                self.paths_m[drone_idx] = self.convert_idx2meter(path)      
                self.smooth_path_m[drone_idx] = self.convert_idx2meter(smooth_path,goal=goal_m,add_end_point=1)
                self.block_volumes_m[drone_idx] = self.convert_idx2meter(self.block_volume[drone_idx])
                return 1
            except:
                print(' No Path Found! agent = '+str( drone_idx)+' from '+str(start_title)+ ' to '+ str(goal_title)+' start:'+str(start_m)+' goal:'+str(goal_m) + ' ')
                print('at_base = ', at_base )
                return 0



