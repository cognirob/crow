#CHECKING PUBLISHING PART OF THE DETECTOR NODE



import rclpy #add to package.xml deps
from rclpy.node import Node
import sensor_msgs
import std_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils
from imutils import perspective
from imutils import contours
from scipy.spatial import distance as dist
#import matplotlib.pyplot as plt

import time

#import commentjson as json
import pkg_resources
import argparse

from std_msgs.msg import String
from sensor_msgs.msg import Image



class CheckSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Image,
            'Cropped',
            self.listener_callback,
            10)
        self.cvb_ = CvBridge()
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I saw:')# "%s"' % msg.data)
        croping = self.cvb_.imgmsg_to_cv2(msg)
        cv2.imshow('Subscribed Crop', croping)

        key = cv2.waitKey(10)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
          cv2.destroyAllWindows()


        


def main(args=None):
    rclpy.init(args=args)

    subscriber = CheckSubscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
