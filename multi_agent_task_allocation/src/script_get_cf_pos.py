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

samples_num = 100
# np.save('current_measure',np.array(61,dtype=int))
def main():
    fc = Flight_manager(1)
    fc.swarm.reset_estimators()
    pos_data = []
    for i in range(samples_num):
        current_pos = fc.get_position(drone_idx=0)
        print(current_pos)
        pos_data.append(current_pos)
        if i%10 == 0:
            print(i)
    fc.swarm.close_links()
    
    pos_data = np.array(pos_data)   
    print('-------------------------------')
    print(f'average position = {np.mean(pos_data, axis=0)}') 
    # np.save('lps_static_pos_optimal_config_30', pos_data)


if __name__ == '__main__':
    main()

