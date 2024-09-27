from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()

tello.go_xyz_speed(100,100,0,10)
tello.go_xyz_speed(-100,-100,0,10)

tello.land()
