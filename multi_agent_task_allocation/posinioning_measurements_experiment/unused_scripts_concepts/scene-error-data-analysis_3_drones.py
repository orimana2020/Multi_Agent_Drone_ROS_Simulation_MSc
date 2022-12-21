# this code takes data collected from envirnment x,y,z,error, process and visualize it

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import os

def load_data(filename):
    # str(os.getcwd())+
    return np.load(str(os.getcwd())+'/'+filename+'.npy')

def get_steady_state_error(raw_data):
    return np.sum(raw_data[:1000,3])/1000

def preprocess_data(raw_data, steady_state_error, threshold_error = 1):
    data_new = np.array([[0,0,0,0]]) # initiate
    for row in raw_data:
        error = row[3] - steady_state_error
        if error < 0:
            error = 0
        if error < threshold_error:
            data_new = np.append(data_new, [[row[0], row[1], row[2], error]], axis=0)
    data_new = np.delete(data_new, [0], axis=0) # delete initiated row
    return data_new

def process_data(data, resolution=0.3):
    x_m_min, x_m_max = np.min(data[:,0]), np.max(data[:,0])
    y_m_min, y_m_max = np.min(data[:,1]), np.max(data[:,1])
    z_m_min, z_m_max = np.min(data[:,2]), np.max(data[:,2])
    size_x = int((x_m_max - x_m_min)/resolution) + 1
    size_y = int((y_m_max - y_m_min)/resolution) + 1
    size_z = int((z_m_max - z_m_min)/resolution) + 1
    space = np.zeros((size_x , size_y , size_z), dtype=float) # row, col, depth
    space_samples_num = np.zeros((size_x , size_y , size_z), dtype=int)

    for sample in data: # sum all error
        x, y, z = int((sample[0] - x_m_min) / resolution), int((sample[1] - y_m_min) / resolution), int((sample[2] - z_m_min) / resolution)
        space[x,y,z] += sample[3]
        space_samples_num[x,y,z] += 1

    valued_space = np.array([[0,0,0,0]]) 
    for x in range(size_x):
        for y in range(size_y):
            for z in range(size_z):
                if space_samples_num[x,y,z] > 0:
                    # average the measurement
                    space[x,y,z] = space[x,y,z] / space_samples_num[x,y,z] 
                    # convet index back to meters
                    x_m, y_m, z_m = (x * resolution + x_m_min) , (y * resolution + y_m_min) , (z * resolution + z_m_min)
                    # create new array of coordinates_meter and error
                    valued_space = np.append(valued_space, [[x_m, y_m, z_m ,space[x,y,z]]], axis=0) 
    valued_space = np.delete(valued_space, [0], axis=0) # [x,y,z, position_error]

    # convert errors to colors , low error - blue,  high error - red
    error_min, error_max = np.min(valued_space[:,3]) ,np.max(valued_space[:,3])
    error_range = error_max - error_min
    factor = (valued_space[:,3] - error_min) / error_range # high error: 1-> red, low error : 0-> blue  
    col = []
    for fac in factor:
        col.append([fac, 0 ,1-fac]) # red,green,blue



    plt.hist(data[:,3])
    lps_anchor_position = np.array([[-1.2,-2,0],[-2.4,-2,2.35],[1.03,2.1,0],[-2.4,2.1,2.35],[1.2,-2,0],[1.03,2.1,2.35],[1.2,-2,2.35],[-2.3,2.1,0]])
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter3D(valued_space[:,0],valued_space[:,1],valued_space[:,2],s=30,c=col)
    ax.scatter3D(lps_anchor_position[:,0],lps_anchor_position[:,1],lps_anchor_position[:,2],s=60,c='green')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim(x_m_min, x_m_max)
    ax.set_ylim(y_m_min, y_m_max)
    ax.set_zlim(z_m_min, z_m_max)
    plt.show()


if __name__ == '__main__':
    filename='first_data_collection_2'
    raw_data = load_data(filename)
    steady_state_error = get_steady_state_error(raw_data)
    print('steady state error = ', steady_state_error)
    prepocessed_data = preprocess_data(raw_data, steady_state_error, threshold_error = 0.5)
    print(f'preprocessed data = {len(prepocessed_data)}, precent of approved data = {len(prepocessed_data)*100/len(raw_data): .2f}')
    print(f'min error = {np.min(prepocessed_data[:,3])} [m], max error = {np.max(prepocessed_data[:,3])} [m]')
    process_data(prepocessed_data, resolution=0.1)
    




# def collect_data(d12gt = 0.5, d23gt = 0.55, d13gt = 0.6, filename='None'):
#     gt_dist_vec = np.array([d12gt , d23gt, d13gt])
#     data = np.array([[0,0,0,0]]) # initiate
#     for i in range(10000):
#         drone1_pos = np.random.rand(1,3) * np.random.randint(0,5) # x y z
#         drone2_pos = np.random.rand(1,3) * np.random.randint(0,5)  # x y z
#         drone3_pos = np.random.rand(1,3) * np.random.randint(0,5)  # x y z
#         d12 = np.linalg.norm(drone1_pos - drone2_pos, ord=2)
#         d23 = np.linalg.norm(drone2_pos - drone3_pos, ord=2)
#         d13 = np.linalg.norm(drone1_pos - drone3_pos, ord=2)
#         dist_vec = np.array([d12, d23, d13])
#         avg_position = (drone1_pos + drone2_pos + drone3_pos)/3 
#         error = np.linalg.norm(gt_dist_vec - dist_vec, ord=2)
#         result = np.append(avg_position, error)
#         data = np.append(data, [result], axis=0)
#     data = np.delete(data, [0], axis=0) # [x,y,z, position_error]
#     np.save(filename, data)

