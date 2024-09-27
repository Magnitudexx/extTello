from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()

#tello.curve_xyz_speed(100,100,0,200,0,0,20)
tello.curve_xyz_speed(-100,-100,0,-200,0,0,20)

tello.land()
