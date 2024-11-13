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
time.sleep(2)
try:
    drone.start_auto_controller(lambda: {'x': 0, 'y': 30, 'z': 90})
    while True:
        continue

finally:
        drone.stop()
