from djitellopy import Tello
import numpy as np
import math

class extTello(Tello):
    def __init__(self):
        super().__init__()
        self.x = 0
        self.path = None
        self.time_array = None
        self.speed_array = None
        self.state = {
            "pos" : [0 ,0 , 0],
            "heading" : 0,
        }

    def path_planner(self, end):
        self.path = interpolate_points(self.x, end, 10) 

    def speed_profile(self,acceleration, deceleration, max_speed, total_time, time_step=0.1):
        self.time_array = np.arange(0, total_time, time_step)
        self.speed_array = np.zeros_like(self.time_array)

        # Phase 1: Acceleration
        for i in range(1, len(self.time_array)):
            if self.speed_array[i - 1] < max_speed:
                self.speed_array[i] = min(max_speed, self.speed_array[i - 1] + acceleration * time_step)
            else:
                self.speed_array[i] = max_speed

        # Phase 2: Braking
        for i in range(1, len(self.time_array)):
            if self.time_array[i] > total_time / 2:  # Start braking after half the total time
                self.speed_array[i] = max(0, self.speed_array[i - 1] - deceleration * time_step)


    def travel_path(self, time_step=0.1):
        if self.path == None or self.speed_array == None:
            print("no path or speed profile available")
            return


        for i in range(1, len(self.path)):
            next_pos = self.path[i]
            current_pos = self.state['pos']

        # Calculate the heading angle needed to go from current_pos to next_pos
            self.state['heading'] = calculate_heading(current_pos, next_pos)

        # Calculate the angle error (difference between current heading and desired heading)
        #heading_error = desired_heading - robot_state['heading']

        # Update the robot's steering by adjusting its heading towards the target
        #if abs(heading_error) > 0.1:  # Threshold to prevent small fluctuations
            # Simple proportional controller for turning
            #steering_adjustment = heading_error * 0.5  # Proportional gain
            #robot_state['heading'] += steering_adjustment * time_step

        # Move the robot forward at the current speed
            current_speed = self.speed_array[i]
            x_speed = current_speed * math.cos(self.state['heading'])
            y_speed = current_speed * math.sin(self.state['heading'])
            self.send_rc_control(x_speed,y_speed,0,0)
            self.state['pos'][0] += x_speed * time_step
            self.state['pos'][1] += y_speed * time_step
        pass
    

def interpolate_points(start, end, num):
    x_values = np.linspace(start, end, num)
    #y_values = np.linspace(start, end ,num)
    #return np.vstack((x_values, y_values)).T
    return np.vstack(x_values).T

def calculate_heading(current_pos, next_pos):
    """Calculates the heading angle (in radians) between the current and next positions."""
    delta_x = next_pos[0] - current_pos[0]
    delta_y = next_pos[1] - current_pos[1]
    return math.atan2(delta_y, delta_x)
