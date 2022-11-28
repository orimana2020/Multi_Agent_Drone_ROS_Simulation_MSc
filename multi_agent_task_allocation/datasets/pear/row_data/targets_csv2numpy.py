import numpy as np
import os
import csv

for i in range(1,21,1):
    with open('FruitdataTree'+str(i)+'.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        targets_x_close = []
        target_x_far = []
        for row in csv_reader:
            x, y, z = float(row[0]), float( row[1]), float(row[2])
            if x <= 0:
                targets_x_close.append([x,y,z])
            else:
                target_x_far.append([x,y,z])
            
        np.save('pear_fruitpos_close_'+str(i)+'.npy', np.array(targets_x_close))
        np.save('pear_fruitpos_far_'+str(i)+'.npy', np.array(target_x_far))
        

    