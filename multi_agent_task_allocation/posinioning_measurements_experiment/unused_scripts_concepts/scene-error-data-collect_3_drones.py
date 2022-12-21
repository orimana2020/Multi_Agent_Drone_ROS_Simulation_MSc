# this function connects to several CF logging data and collect CF relative positions
# the output is numpy array with each row represents [x,y,z,error] 
# where x,y,z are coords of average drones position
# and error is differance between measured distance of drone relative to ground truth


import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
import numpy as np

def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')

uri_1 = 'radio://0/80/2M/E7E7E7E7E1'
uri_2 = 'radio://0/80/2M/E7E7E7E7E2'
uri_3 = 'radio://0/80/2M/E7E7E7E7E3'
uri_4 = 'radio://0/80/2M/E7E7E7E7E4'

mode = 'debug' # debug single drone
# mode = 'collect' # For collecting data from multiple drones

uris = {uri_1, uri_2, uri_3}
uri_l = [uri_1, uri_2, uri_3]
if mode == 'debug':
    uris = {uri_1}
    uri_l = [uri_1]


drone_pos = []
for uri in uris:
    drone_pos.append([])

# set ground truth distance between drones
d01gt = 0.865 # 1-2
d12gt = 0.84 # 2-3
d02gt = 0.65 # 1-3 
gt_dist_vec = np.array([d01gt , d12gt, d02gt])
filename='first_data_collection_3'
data = np.array([[0,0,0,0]]) # initiate
sample_time = 240 # [sec]
samples = 30000
# sleep time = sample_time / samples

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.reset_estimators()
        # scf = swarm._cfs['radio://0/80/2M/E7E7E7E7E7']
        for i in range(samples):
            swarm.get_estimated_positions() #position from kalman
            print(i)
            for link_idx in range(len(uri_l)):
                x,y,z = swarm._positions[uri_l[link_idx]]
                print(f'drone: {link_idx}, x ={x: .3f}, y ={y: .3f}, z ={z: .3f}')
                drone_pos[link_idx] = np.array([x,y,z])
            if mode != 'debug':  
                d01 = np.linalg.norm(drone_pos[0] - drone_pos[1], ord=2)
                d12 = np.linalg.norm(drone_pos[1] - drone_pos[2], ord=2)
                d02 = np.linalg.norm(drone_pos[0] - drone_pos[2], ord=2)
                dist_vec_measured = np.array([d01, d12, d02])
                avg_position = (drone_pos[0] + drone_pos[1] + drone_pos[2]) / 3 
                error = np.linalg.norm(gt_dist_vec - dist_vec_measured, ord=2)
                result = np.append(avg_position, error)
                data = np.append(data, [result], axis=0)
                print('error = ', error)
            time.sleep(0.2)
        if mode != 'debug':
            data = np.delete(data, [0], axis=0) # delete initiated row
            np.save(filename, data)

