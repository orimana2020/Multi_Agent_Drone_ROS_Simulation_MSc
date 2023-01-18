import numpy as np
import params
from sklearn.cluster import KMeans
from itertools import combinations

drone_num = 3
targetpos = params.targetpos
# kmeans = KMeans(n_clusters=3).fit(targetpos[:,1:]) # ------------- change to yz dist
# centers = kmeans.cluster_centers_
# print(centers)

# downwash_distance = params.downwash_distance
# a0 = -downwash_distance[2][0]/downwash_distance[1][0] 
# b0 = downwash_distance[2][0] 
# centers = np.array([[0,0],[-0.1,0.2],[2,2]])


# combs = list(combinations(list(range(drone_num)), 2))
# for comb in combs:
#     print(comb)
#     y1,z1 = centers[comb[0],0], centers[comb[0],1]
#     y2,z2 = centers[comb[1],0], centers[comb[1],1]
#     dy = abs(y1-y2)
#     dz = abs(z1-z2)
#     if dy <= downwash_distance[1][0] and dz <= downwash_distance[2][0]:
#         z = a0*dy + b0
#         if dz <= z: #inside
           
#             print('inside')
#             # return True

# a = np.array([1,2,3,4,5])
# print(np.average(a))
# print(np.linalg.norm(a, ord=2))
# y_target_idx = [2,5,1,3,7]
# y_targets = np.array([1,2,3,4,-5])

# drone_idx = np.array([0,1,2,3,4])
# y_drones = np.array([1,2,3,4,-10])

# #sort targets
# sorted_targets_indices = np.argsort(y_targets) 
# # y_targets[:] = [y_targets[i] for i in sorted_targets_indices]
# y_target_idx[:] = [y_target_idx[i] for i in sorted_targets_indices]

# # sort drones
# indices_y_drones = np.argsort(y_drones)
# # y_drones[:] = [y_drones[i] for i in indices_y_drones]
# drone_idx[:] = [drone_idx[i] for i in indices_y_drones]

# # sort by drone idx
# indeces_drone_idx = np.argsort(drone_idx)
# drone_idx[:] = [drone_idx[i] for i in indeces_drone_idx]
# # y_drones[:] = [y_drones[i] for i in indeces_drone_idx]
# # y_targets[:] = [y_targets[i] for i in indeces_drone_idx]
# y_target_idx[:] = [y_target_idx[i] for i in indeces_drone_idx]

# print(drone_idx)
# print(y_target_idx)

s = 'orikirbi'

n=2
# for i in range(len(s)-1):
#     s[i],s[i+1] = s[i+1],s[i]
