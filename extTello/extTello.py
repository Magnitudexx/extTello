import time
from djitellopy import Tello
import numpy as np
import math

class extTello(Tello):
    def __init__(self):
        super().__init__()
        self.state = {
            "pos" : [0.0, 0.0, 0.0],
            "heading" : [0.0, 0.0],
        }


    def __calc_speed_profile(self,dist,acc,max_v=50.0):
            # Time to reach max speed
        time_to_max_v = float(max_v / acc)
        # dist to reach max speed and to decelerate
        dist_acc_dec = 0.5 * acc * time_to_max_v ** 2
        
        if dist < (2 * dist_acc_dec):
            # Short dist case: No constant speed, just accelerate then decelerate
            time_accel = float(np.sqrt(dist / acc))
            max_s = acc * time_accel;
            return [float(acc*t) for t in np.arange(0, time_accel, 0.1)] + \
                   [float(max_s  - acc * t) for t in np.arange(0, time_accel, 0.1)]
        else:
            # Long dist case: Accelerate, then maintain speed, then decelerate
            return [float(acc*t) for t in np.arange(0, time_to_max_v, 0.1)] + \
                   [max_v for t in np.arange(time_to_max_v, dist / max_v, 0.1)] + \
                   [float(max_v - acc * (t - dist / max_v)) for t in np.arange(dist / max_v, time_to_max_v + dist / max_v, 0.1)]



    def travel_path(self, wps, max_speed, acc):
        for i in range(len(wps) - 1):
            start = wps[i]
            end = wps[i + 1]
            
            # Compute distance and direction to next waypoint
            dist = self.__distance(start, end)
            dir_x, dir_y, dir_z = (end[0] - start[0]) / dist, (end[1] - start[1]) / dist, (end[2] - start[2]) / dist
            
            # Get speed profile for this segment
            speed_profile = self.__calc_speed_profile(float(dist), acc, max_speed)
            print(speed_profile) 
            # Move self along speed profile
            for speed in speed_profile:
                self.send_rc_control(int(speed * dir_x), int(speed * dir_y), int(speed * dir_z), 0)
                time.sleep(0.1)
            
            # Stop between wps to stabilize
            self.send_rc_control(0, 0, 0, 0)
            time.sleep(2)

    def __distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)
