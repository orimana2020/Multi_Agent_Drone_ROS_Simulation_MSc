import time
from sklearn.cluster import KMeans
import params
import matplotlib.pyplot as plt
from itertools import combinations


fig1 = plt.figure()
ax1 = fig1.add_subplot('111')

fig2 = plt.figure()
ax2 = fig2.add_subplot('111')

targetpos_ = params.targetpos
targetpos = targetpos_[:,1:]
n=3

kmeans = KMeans(n_clusters=n).fit(targetpos)
centers = kmeans.cluster_centers_
ax1.scatter(targetpos[:,0], targetpos[:,1])
ax1.scatter(centers[:,0], centers[:,1], s=30)

kmeans = KMeans(n_clusters=n*2).fit(targetpos)
centers = kmeans.cluster_centers_
ax2.scatter(targetpos[:,0], targetpos[:,1])
ax2.scatter(centers[:,0], centers[:,1], s=30)

print(list(combinations(list(range(2*n)), n)))
# plt.show()