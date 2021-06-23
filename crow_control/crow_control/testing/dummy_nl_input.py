import os
import json
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from crow_msgs.msg import SentenceProgram

class DummyNlInput(Node):

    def __init__(self, node_name="dummy_nl_input"):
        super().__init__(node_name)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.nl_publisher = self.create_publisher(SentenceProgram, "/nl_input", qos)
        self.publish_command()

    def publish_command(self):

        recog_text = 'pustit'
        msg = SentenceProgram()
        msg.data.append(recog_text)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.nl_publisher.publish(msg)
        print(f'published {msg.data}')

def main():
    rclpy.init()
    time.sleep(1)
    dummy_nl = DummyNlInput()
    rclpy.spin(dummy_nl)
    dummy_nl.destroy_node()

if __name__ == "__main__":
    main()