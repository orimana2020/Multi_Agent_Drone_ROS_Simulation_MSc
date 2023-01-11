import numpy as np

a = np.load('pear_fruitpos_close_1offset_2.3_-0.75_0.npy')
b = np.load('pear_fruitpos_close_2offset_2.3_-0.75_0.npy')
c = np.vstack((a,b))
np.save('experiment_1_targets', c)