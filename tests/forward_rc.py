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
    while True:
        drone.send_rc_control(20, 0, 0, 0)
        time.sleep(0.05)

finally:
        drone.stop()
