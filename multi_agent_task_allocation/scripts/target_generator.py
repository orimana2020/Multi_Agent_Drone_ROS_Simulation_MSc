import numpy as np
import math
targets_num_gen = 30
t = np.linspace(0, 2*np.pi-2*np.pi/targets_num_gen, targets_num_gen)
radius = 0.6
depth = 2.4
floor_safety_distance = 0.8
z_offset = radius + floor_safety_distance + 0.3
targetpos = np.stack([depth*np.ones([targets_num_gen]) , radius * np.cos(t), radius * np.sin(t) + z_offset] , axis=-1)


for i in range(len(targetpos)):
    print("<include>")
    print("  <uri>model://flower2</uri>")
    print('  <name>target'+str(i)+'</name>')
    print('  <pose>'+str(targetpos[i][0])+(' ')+str(targetpos[i][1])+(' ')+str(targetpos[i][2])+' 0 -1.57 0</pose>')
    print("</include>")
   