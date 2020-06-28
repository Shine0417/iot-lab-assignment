import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import matplotlib.pyplot as plt
import cv2
import numpy as np
import matplotlib as mpl
import sys


class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)

        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 10)

        self.tmr = self.create_timer(0.3, self.controller_callback)
        self.steer= 12
        self.throttle = 9
        self.brake = 0

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = int(self.throttle)
        msg.front_steer = int(self.steer)
        msg.brake = int(self.brake)
        self.pub.publish(msg)

        
    def bounding_boxes_callback(self, data):
        print('There are %d bounding boxes.' % len(data.boxes))
        self.throttle = 9
        self.brake = 0
        self.steer = 12
        for box in data.boxes:
            if abs(box.size.x - box.size.y) < 0.95 and box.size.x < 3.15 and box.size.x > 2.1:
                #print('central  %0.3f %0.3f %0.3f, size %0.3f %0.3f %0.3f.' %(box.centroid.x, box.centroid.y, box.centroid.z, box.size.x, box.size.y, box.size.z))
                print('road light detected!')
                # detect road light
                if box.centroid.x < 3:
                    self.brake = 80
                    self.throttle = 0
                self.steer = 12

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


