import numpy as np
import params
from sklearn.cluster import KMeans
from itertools import combinations

drone_num = 3
targetpos = params.targetpos
# kmeans = KMeans(n_clusters=3).fit(targetpos[:,1:]) # ------------- change to yz dist
# centers = kmeans.cluster_centers_
# print(centers)

downwash_distance = params.downwash_distance
a0 = -downwash_distance[2][0]/downwash_distance[1][0] 
b0 = downwash_distance[2][0] 
centers = np.array([[0,0],[-0.1,0.2],[2,2]])


combs = list(combinations(list(range(drone_num)), 2))
for comb in combs:
    print(comb)
    y1,z1 = centers[comb[0],0], centers[comb[0],1]
    y2,z2 = centers[comb[1],0], centers[comb[1],1]
    dy = abs(y1-y2)
    dz = abs(z1-z2)
    if dy <= downwash_distance[1][0] and dz <= downwash_distance[2][0]:
        z = a0*dy + b0
        if dz <= z: #inside
           
            print('inside')
            # return True