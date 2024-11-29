from extTello import extTello
import time
import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    handlers=[logging.FileHandler("thread_logs.log"), logging.StreamHandler()]
)
drone = extTello()

drone.connect()
drone.takeoff_with_state()
try:
    drone.go_xyz_speed(0, 30, 0, 20)

finally:
        drone.stop()
