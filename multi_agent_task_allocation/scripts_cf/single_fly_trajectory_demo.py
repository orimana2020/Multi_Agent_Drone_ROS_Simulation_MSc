
import sys
import time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.Ori_CF.multi_agent_task_allocation.src.CF_Trajectory import Generate_Trajectory




class Uploader:
    def __init__(self):
        self._is_done = False
        self._success = True

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done, write_failed_cb=self._upload_failed)

        while not self._is_done:
            time.sleep(0.2)

        return self._success
        
    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True
        self._success = True

    def _upload_failed(self, mem, addr):
        print('Data upload failed')
        self._is_done = True
        self._success = False


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)
            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)
            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf,trajectory_id ,trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    upload_result = Uploader().upload(trajectory_mem)
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

def execute_trajectory(commander, wp):
    try:
        trajectory_id = 1
        traj = Generate_Trajectory(wp, velocity=1, plotting=0)
        traj_coef = traj.poly_coef
        duration = upload_trajectory(cf, trajectory_id ,traj_coef)
        commander.start_trajectory(trajectory_id, 1.0, False)
        time.sleep(duration)
    except:
        pass



def run_sequence(cf):
    #take off
    commander = cf.high_level_commander
    commander.takeoff(1.0, 2.0)
    time.sleep(2.0)  

    for _ in range(5):
    # go forward
        execute_trajectory(commander, wp = get_original_wp_line())
        # go backward
        execute_trajectory(commander, wp = get_original_wp_line(is_reversed=True))

    # land
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def get_original_wp_line(is_reversed=False):
    wp_orig = []
    for x in range(10):
        wp_orig.append([-0.5+0.1*x, -1-x*0.1, 1]) 
    wp_orig = np.array(wp_orig)
    if is_reversed:
        return wp_orig[::-1]
    return wp_orig


uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        activate_high_level_commander(cf)
        # activate_mellinger_controller(cf)
        reset_estimator(cf)
        run_sequence(cf)
        
        
        