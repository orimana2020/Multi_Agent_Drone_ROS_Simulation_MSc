import numpy as np
import os

# -------------------- INSTRUCTIONS

# To save txt file:
# 1. cd to scripts folder in terminal
# 2. run from terminal: python3 target_generator_real_tree.py > target.txt
# ------------------------------
x_offset = 4 #[m]
y_offset = 0 #[m]
z_offset = 0 #[m]

file_name = 'pear_fruitpos_close_1'
targets  = np.load(str(os.getcwd())+'/pear/row_data/'+file_name+'.npy') # data in cm
targets = targets / 100  # convert to [m]
targets += np.array([x_offset, y_offset, z_offset]) # add offset [m]
np.save(file_name+'offset_'+str(x_offset)+'_'+str(y_offset)+'_'+str(z_offset)+'.npy', targets)
i = 0
for row in targets:
    print("<include>")
    print("  <uri>model://flower2</uri>")
    print('  <name>target'+str(i)+'</name>')
    print('  <pose>'+str(row[0])+(' ')+str(row[1])+(' ')+str(row[2])+' 0 -1.57 0</pose>')
    print("</include>")
    i += 1
    
    
