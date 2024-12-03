from extTello import extTello
import time
import logging
import rclpy

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    handlers=[logging.FileHandler("thread_logs.log"), logging.StreamHandler()]
)

rclpy.init()
drone = extTello()

drone.init_ros_publisher()
drone.connect()
drone.takeoff_with_state()
try:
    drone.publish_data("hello")

finally:
        drone.stop()
