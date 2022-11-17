import numpy as np
import random
import os
from os.path import dirname, join as pjoin
import scipy.io as sio


for d in range(10,11):
    for k in range(2,4):
        data_dir = pjoin(os.getcwd())
        mat_fname = pjoin(data_dir, 'state_mat_d'+str(d)+'_k'+str(k)+'.mat')
        mat_contents = sio.loadmat(mat_fname)
        mat_contents = np.array(mat_contents['state_matrix'], dtype=np.int32)
        np.save(str(os.getcwd())+'/state_mat/state_mat_d'+str(d)+'_k'+str(k), mat_contents)
        #state_mat = np.load('state_mat_d'+str(6)+'_k'+str(8)+'.npy')

