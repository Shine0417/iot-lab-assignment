import rclpy
from rclpy.node import Node
import numpy as np
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

def start():
    #return throttle, brake, front_steer
    return 20, 0, 0 
