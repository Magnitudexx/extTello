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
    drone.travel_path([{'x' : 0, 'y' : 0}, {'x' : 120, 'y' : 0}], 20, 4)

finally:
        drone.stop()
