
from sklearn.cluster import KMeans
import params
drone_num = params.drone_num
targetpos = params.targetpos
kmeans = KMeans(n_clusters=drone_num).fit(targetpos)
centers = kmeans.cluster_centers_

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig, ax = plt.subplots()
fig = plt.figure(1)
# self.ax = self.fig.add_subplot(111)#, projection='3d')
ax =  Axes3D(fig)
ax.scatter(targetpos[:,0], targetpos[:,1],targetpos[:,2], 'r-')
ax.scatter(centers[:,0], centers[:,1],centers[:,2])


ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim([1.4,3.6])
ax.set_ylim([-1,1])
ax.set_zlim([0.4,2.4])

plt.show()