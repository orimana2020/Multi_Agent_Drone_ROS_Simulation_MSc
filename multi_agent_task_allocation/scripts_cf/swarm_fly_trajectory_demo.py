import time
import cflib.crtp
import numpy as np
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.Ori_CF.multi_agent_task_allocation.src.CF_Trajectory import Generate_Trajectory, upload_trajectory


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')

def take_off(scf):
    cf = scf.cf
    commander = cf.high_level_commander
    commander.takeoff(1.0, 2.0)
    time.sleep(2.0) 

def execute_trajectory(scf, waypoints): 
    cf = scf.cf
    commander = cf.high_level_commander 
    x, y, z = waypoints[0]
    print('start wp = ', waypoints[0])
    commander.go_to(x, y, z, yaw=0, duration_s=3)
    time.sleep(3)
    try:
        trajectory_id = 1
        traj = Generate_Trajectory(waypoints, velocity=1, plotting=0, force_zero_yaw=False)
        traj_coef = traj.poly_coef
        duration = upload_trajectory(cf, trajectory_id ,traj_coef)
        commander.start_trajectory(trajectory_id, 1.0, False)
        time.sleep(duration)
    except:
        print('failed to execute trajectory')
    
def land(scf):
    cf = scf.cf
    commander = cf.high_level_commander
    commander.land(0.0, 4.0)
    time.sleep(4)
    commander.stop()


# --------------------------- Examples --------------------------------

def get_wp_circle(is_reversed=False):
    wp_orig = []
    radius = 1
    angles = np.linspace(0,np.pi, 20)
    z=1
    for i in angles:
        z += 0.05
        wp_orig.append([-0.5+radius*np.sin(i),radius*np.cos(i), z]) 
    wp_orig = np.array(wp_orig)
    if is_reversed:
        return wp_orig[::-1]
    return wp_orig


def get_wp(offset, is_reversed=False):
    x_off,y_off = offset
    wp_orig = []
    for x in range(10):
        wp_orig.append([x_off+0.1*x, y_off+x*0.1, 1]) 
    wp_orig = np.array(wp_orig)
    if is_reversed:
        return wp_orig[::-1]
    return wp_orig



uri1 = 'radio://0/80/2M/E7E7E7E7E1'
uri2 = 'radio://0/80/2M/E7E7E7E7E2'
uri3 = 'radio://0/80/2M/E7E7E7E7E3'
uri4 = 'radio://0/80/2M/E7E7E7E7E4'
uri_list = [uri1]
uris = set(uri_list)
print(uris)
open_threads = []

if __name__ == '__main__':
    # init mission
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    swarm = Swarm(uris, factory=factory)
    swarm.open_links()
    swarm.parallel_safe(activate_high_level_commander)
    swarm.reset_estimators()

    # mission
    swarm.parallel_safe(take_off)
    t1 = swarm.trajectory_to_drone(execute_trajectory, uri1, waypoints= get_wp_circle(is_reversed=False))
    open_threads.append(t1)
    # t2 = swarm.trajectory_to_drone(execute_trajectory, uri1, waypoints= get_wp(offset=(-0.35,-1),is_reversed=True))
    # open_threads.append(t2)

    thread_running = True
    while thread_running : # check if all threads are finished before sending new commands
        thread_running = False
        for thread in open_threads:
            if thread.is_alive():
                thread_running = True
        # print('main thread is running in background')
        time.sleep(0.1)
    open_threads = []      
        
    # end mission
    swarm.parallel_safe(land)
    swarm.close_links()

