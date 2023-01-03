from planner_3D import Trajectory
from Allocation_algorithm import Allocation
import matplotlib.pyplot as plt
from Additionals import get_figure , Drone_Manager
import numpy as np
import params
import time
from CF_Flight_Manager import Flight_manager



ta = Allocation() # compute fisrt targets allocation
fig = get_figure()
dm = Drone_Manager(params.uri_list, params.base, params.magazine, ta)

fc = Flight_manager(ta.drone_num)
fc.take_off_swarm()

fc.go_to(0,[1, 0 ,1])
fc.go_to(1,[1, 0 ,0.5])
time.sleep(5)



fc.go_to(0,[1, 0.7 ,1])
# fc.go_to(1,[1.6, 0 ,1])
time.sleep(2)

for i in range(1000):
    fc.land('all', dm.drones)
    time.sleep(0.1)