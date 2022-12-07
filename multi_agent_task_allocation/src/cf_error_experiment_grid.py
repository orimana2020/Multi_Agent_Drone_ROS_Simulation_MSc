#! /usr/bin/env python3
import numpy as np
import params
import time

if params.mode == 'sim':
    from rotors_flight_manager import Flight_manager
    import rospy
    rospy.init_node('send_my_command', anonymous=True)
    rospy.sleep(3)
elif params.mode == 'cf':
    from CF_Flight_Manager import Flight_manager

x_min, x_max , y_min, y_max, z_min, z_max = [0,0.6, -0.3,0.3, 0.3,1]
samples_num = 10
min_bat_voltege = params.min_battery_voltage
check_battery_interval_time = params.check_battery_interval_time

def get_coords(c_min, c_max, step_size=0.2):
    x = c_min
    coords = []
    while x <= c_max:
        coords.append(x)
        x += step_size
    return coords, coords[::-1]



def main():
    start_time = time.time()
    delta_time = time.time()
    fc = Flight_manager(1)
    fc.take_off_swarm()
    x_for, x_rev = get_coords(x_min, x_max)
    y_for, y_rev = get_coords(y_min, y_max)
    z_for, _ = get_coords(z_min, z_max)
    points_to_check = len(x_for)*len(y_for)*len(z_for)
    counter_points  = 0
    break_flag = False
    pos_data = []
    print(f'total points to check:{points_to_check}')
    manage = [0,0,0]
    z_cur = z_for
    for z_val in z_cur:
        if break_flag:
            break
        if manage[2] == 0:
            y_cur = y_for
            manage[2] = 1
        else:
            y_cur = y_rev
            manage[2] = 0
        for y_val in y_cur:
            if break_flag:
                break
            if manage[1] == 0:
                x_cur = x_for
                manage[1] = 1
            else:
                x_cur = x_rev
                manage[1] = 0
            for x_val in x_cur:
                samples = 0
                goal = [x_val, y_val, z_val]
                print(f'current_goal = {goal}')
                while samples < samples_num:
                    if fc.reached_goal(goal=goal, drone_idx=0):
                        pos_data.append(fc.get_position(drone_idx=0))
                        print('#' , end= '' )
                        samples += 1
                    elif not fc.open_threads[0].is_alive():
                        fc.go_to(drone_idx=0, goal=goal)
                    fc.sleep()
                print()
                counter_points += 1
                print(f'process progerss: {int(counter_points* 100 / points_to_check)} ')
                # safety break for low battery

                print(f'battery level = {fc.get_battery(drone_idx=0)}')
                if  time.time() - delta_time  > check_battery_interval_time:
                    delta_time = time.time()
                    battery_vol = fc.get_battery(drone_idx=0)
                    if battery_vol < min_bat_voltege:
                        print(f'LOW BATTERY, last goal = {goal}')
                        break_flag = True
                        break
                    
    print(f'total flyling time = {time.time() - start_time}')                
    time.sleep(5)
    fc.land(drone_idx=0)
    np.save('cf_postion_errors_experiment2', np.array(pos_data))
                
if __name__ == '__main__':
    main()

