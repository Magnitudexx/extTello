import time
from djitellopy import Tello
import logging
import numpy as np
import math
import threading
import time
from typing import Callable, Dict
from types import MethodType

class extTello(Tello):
    def __init__(self, x=0.0, y=0.0, z=0.0, theta=0.0):
        super().__init__()
        self.running = False
        self.state = {'x': x, 'y': y, 'z':z}
        self.heading = theta
        self.StateUpdaterThread = threading.Thread(target=self.StateUpdater)
        self.auto_controller_thread = None
        self.lock = threading.Lock()
        self.frame = None
        self.mask = None
        self.have_identifier = False

    def StateUpdater(self):
        lt = time.time()

        while self.running:
            with self.lock:
                vx = self.get_speed_x();
                vy = self.get_speed_y();
                vz = self.get_speed_z();
            logging.debug(f"Speeds - vx: {vx}, vy: {vy}, vz: {vz}")

            dt = (time.time() - lt)
            logging.debug(f"Delta time : {dt}")
            with self.lock:
                self.state['x'] = self.state['x'] +(10 * vx *dt)
                self.state['y'] = self.state['y'] +(10 * vy *dt)
                self.state['z'] = self.get_distance_tof()

            logging.debug(f"State : {self.state}")

            lt = time.time()
            time.sleep(0.1)




    def travel_path(self, wps, max_speed, acc):
        idn = getattr(self, "object_identifier", lambda: None)
        for i in range(len(wps) - 1):
            start = wps[i]
            end = wps[i + 1]
            
            # Compute distance and direction to next waypoint
            dist = self.__distance(start, end)
            #dir_x, dir_y, dir_z = (end[0] - start[0]) / dist, (end[1] - start[1]) / dist, (end[2] - start[2]) / dist
            dir_x, dir_y = (end['x'] - start['x']) / dist, (end['y'] - start['y']) / dist
            
            # Get speed profile for this segment
            speed_profile = self.__calc_speed_profile(float(dist), acc, max_speed)
            # Move self along speed profile
            for speed in speed_profile:
                if idn() is not None:
                    self.send_rc_control(0,0,0,0)
                    self.start_auto_controller()
                    return
                self.send_rc_control(int(speed * dir_x), int(speed * dir_y), 0, 0)
                time.sleep(0.05)
            
            # Stop between wps to stabilize
            self.send_rc_control(0, 0, 0, 0)
            time.sleep(2)
    def start_auto_controller(self):
        if self.have_identifier:
            self.auto_controller_thread = threading.Thread(target=self.__auto_controller)
            self.auto_controller_thread.start()

    def stop(self):
        self.running = False
        self.StateUpdaterThread.join()
        self.auto_controller_thread.join()
        self.land()
        self.end()

    def takeoff_with_state(self):
        self.takeoff()
        self.running = True
        self.StateUpdaterThread.start()
        time.sleep(1)
        try:
            self.move_forward(20)
            time.sleep(1)
            self.move_up(20)
            time.sleep(1)
        except:
            return
        """
        while self.state['z'] < 110:
            self.send_rc_control(0,0,10,0)
            time.sleep(0.05)
        self.send_rc_control(0,0,0,0)
        time.sleep(1)
        """
    def create_object_identifier(self, func):
        self.have_identifier = True
        setattr(self, "object_identifier", MethodType(func, self))

    def __auto_controller(self):
        func = getattr(self, 'object_identifier', lambda: None)
        lim = 20
        while self.running:
            target = func()
            if target is None:
                continue
            logging.debug(f"Target = {target}")
            #if 'z' not in target:
                #target['z'] = pos['z']
            pos = self.state
            dist = self.__distance(pos, target)
            logging.debug(f"Dist = {dist}")
            #dx = int(target['x'] - pos['x']) if  int(target['x'] - pos['x'])  >= lim else 0
            #dy = int(target['y'] - pos['y']) if  int(target['y'] - pos['y'])  >= lim else 0
            dx = 20 if  int(target['x'])  >= lim else 0
            dy = 20 if  int(target['y'])  >= lim else 0
            logging.debug(f"dx, dy = {dx}, {dy}")

                    # If close enough to the target, hold position
            if dist <= lim:
                logging.debug("Target reached. Holding position.")
                self.send_rc_control(0, 0, 0, 0)
            else:
                self.send_rc_control(dx, dy, 0, 0)

            # Add a small delay to reduce loop frequency
            time.sleep(0.1)


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
                   [max_v for _ in np.arange(time_to_max_v, dist / max_v, 0.1)] + \
                   [float(max_v - acc * (t - dist / max_v)) for t in np.arange(dist / max_v, time_to_max_v + dist / max_v, 0.1)]

    def __distance(self, point1, point2):
        #return np.sqrt((point1['x'] - point2['x']) ** 2 + (point1['y'] - point2['y']) ** 2 + (point1['z'] - point2['z']) ** 2)
        return np.sqrt((point1['x'] - point2['x']) ** 2 + (point1['y'] - point2['y']) ** 2 )
