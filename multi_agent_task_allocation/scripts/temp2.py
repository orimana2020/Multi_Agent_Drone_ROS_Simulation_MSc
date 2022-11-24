from itertools import product, combinations
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
# plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Make data
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = np.outer(np.cos(u), np.sin(v))
y = np.outer(np.sin(u), np.sin(v))
z = np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface
ax.plot_surface(x, y, z)

# #draw cube
# r = [-1, 1]
# for s, e in combinations(np.array(list(product(r,r,r))), 2):
#     if np.sum(np.abs(s-e)) == r[1]-r[0]:
#         ax.plot3D(*zip(s,e), color="b")


# Make axes limits 
# xyzlim = np.array([ax.get_xlim3d(),ax.get_ylim3d(),ax.get_zlim3d()]).T
# XYZlim = [min(xyzlim[0]),max(xyzlim[1])]
ax.set_xlim3d([0,3])
ax.set_ylim3d([-0.6,0.6])
ax.set_zlim3d([0,2.2])
# try:
#     ax.set_aspect('equal')
# except NotImplementedError:
#     pass

# ax.set_title(f"{matplotlib.__version__}")
# fig.canvas.flush_events()
plt.show()