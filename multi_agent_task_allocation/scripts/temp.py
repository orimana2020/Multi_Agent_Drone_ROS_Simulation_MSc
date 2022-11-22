import numpy as np
from scipy import interpolate

def get_smooth_path(path,len1,len3):
    # s = smoothness, m > k must hold, default k degree is  k=3, m is number of points
    weights = np.ones(len(path))*10
    weights[0:len1] = 100
    weights[len(path)-len3:] = 100
    print(weights)
    tck, _ = interpolate.splprep([path[:,0], path[:,1], path[:,2]],w=weights,s=10)  
    # x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0,1,30) # determine number of points in smooth path 
    smooth_path = interpolate.splev(u_fine, tck)
    return np.transpose(np.array(smooth_path))



path = [[1.49689998, 0.69919303 ,0.40406128],
 [1.59606345, 0.70102183 ,0.39706879],
 [1.68986167, 0.70054682 ,0.39696872],
 [1.77858503 ,0.69810718, 0.40343054],
 [1.8625239 , 0.69404205, 0.41612376],
 [1.94196867 ,0.68869062, 0.43471785],
 [2.01720974, 0.68239205, 0.4588823 ],
 [2.08853749, 0.67548551, 0.4882866 ],
 [2.15624231 ,0.66831015, 0.52260023],
 [2.22061457, 0.6612051, 0.56149268],
 [2.28194468, 0.65450969, 0.60463345],
 [2.34052301, 0.64856291 ,0.651692  ],
 [2.39663996 ,0.64370398, 0.70233784],
 [2.4505859  ,0.64027209, 0.75624045],
 [2.50265128 ,0.63860573, 0.81306924],
 [2.55314786 ,0.63876223, 0.87246388],
 [2.60244192 ,0.64007996, 0.93398791],
 [2.65090835 ,0.641784  , 0.99719289],
 [2.69892203 ,0.64309945, 1.06163039],
 [2.74690124 ,0.64328191 ,1.12678314],
 [2.79611684 ,0.64218647, 1.19078165],
 [2.84861838 ,0.64021571, 1.25052147],
 [2.90648401 ,0.63779233 ,1.30285276],
 [2.97179186 ,0.63533901 ,1.34462573],
 [3.04611935 ,0.63323237 ,1.37342112],
 [3.1284767  ,0.63161271 ,1.39056544],
 [3.21705363 ,0.6305448  ,1.39858233],
 [3.31003949 ,0.6300934 , 1.39999598],
 [3.40562364 ,0.63032326, 1.39733057],
 [3.50199541 ,0.63129911, 1.39311029]]


len1=7
len3=9
delta = 0

path = np.array(path)
print(path)
smoooth_path = get_smooth_path( path,len1, len3)


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig, ax = plt.subplots()
fig = plt.figure(1)
# self.ax = self.fig.add_subplot(111)#, projection='3d')
ax =  Axes3D(fig)
ax.scatter(path[:,0], path[:,1],path[:,2], 'r-')
ax.plot(smoooth_path[:,0], smoooth_path[:,1],smoooth_path[:,2])

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim([1.4,3.6])
ax.set_ylim([-1,1])
ax.set_zlim([0.4,2.4])

plt.show()