import numpy as np
import sys
import os
sys.path.insert(0, os.getcwd()+'/src/rotors_simulator/multi_agent_task_allocation/src/' )
import params


def generate_fake_error_mapping(): 
    x_min,x_max,y_min,y_max,z_min,z_max = params.limits_idx
    y_max = round(y_max - y_min) 
    y_min = 0
    res = params.resolution
    worst_accuracy = 0.5 / res
    best_accuracy = 0.05 / res
    error_arr = np.zeros([z_max-z_min,y_max-y_min, x_max-x_min], dtype=int)
    y_middle = round((y_min+y_max)/2)
    for x in range(x_max):
        for y in range(y_max):
            for z in range(z_max):
                dist_x = x
                dist_y = abs(y - y_middle)
                total_dist_score_x = (dist_x / x_max) 
                total_dist_score_y = (np.exp(abs(y-y_middle)/y_middle) - 1) / (np.exp(1) - 1)
                total_dist_score = (total_dist_score_x + total_dist_score_y) / 2
                error_arr[z,y,x] = round(total_dist_score * (worst_accuracy - best_accuracy) + best_accuracy)
    return error_arr


            

            


