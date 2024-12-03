import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROSPublisherNode(Node):
    def __init__(self, topic_name):
        super().__init__('tello_ros_publisher')
        self.publisher = self.create_publisher(String, topic_name, 10)

    def publish(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
