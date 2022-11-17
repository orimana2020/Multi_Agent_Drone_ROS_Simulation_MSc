import numpy as np
import math
import pandas as pd
import os

# -------------------- INSTRUCTIONS

# To save txt file:
# 1. cd to scripts folder in terminal
# 2. run from terminal: python target_generator_real_tree.py > target.txt
# ------------------------------
x_offset = 4
y_offset = -2
z_offset = 0
rejected = 0

targets  = np.empty((0,3), float)
for i in range(3):
    data = pd.read_csv('FruitdataTree'+str(i+1)+'.csv')
    data = np.array(data)
    data = data/100
    [rows,_] = data.shape
    for row in range(rows):
        if data[row][0] < 0:
            targets = np.append(targets, np.array([[data[row][0]+x_offset,data[row][1]+y_offset, data[row][2]+z_offset]]), axis=0)
        else:
            rejected += 1
np.save('targets_arr',targets) # save the array and load it
print(max(targets[:,0])-min(targets[:,0]))

targets_num,_ = targets.shape
for row in range(targets_num):
    print("<include>")
    print("  <uri>model://flower2</uri>")
    print('  <name>target'+str(row)+'</name>')
    print('  <pose>'+str(targets[row,0])+(' ')+str(targets[row,1])+(' ')+str(targets[row,2])+' 0 -1.57 0</pose>')
    print("</include>")
    
    
