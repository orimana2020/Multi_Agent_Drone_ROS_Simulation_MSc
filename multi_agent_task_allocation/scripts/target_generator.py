import numpy as np
import math
import csv
targets = 15
t = np.linspace(0, math.pi*2-math.pi*2/targets , targets)
r=1
x_dist = 3
const_h = r+0.5

for i in range(targets):
    print("<include>")
    print("  <uri>model://flower2</uri>")
    print('  <name>target'+str(i)+'</name>')
    print('  <pose>'+str(x_dist)+(' ')+str(math.cos(t[i]))+(' ')+str(math.sin(t[i])+const_h)+' 0 -1.57 0</pose>')
    print("</include>")
   