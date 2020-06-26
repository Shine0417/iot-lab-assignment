import rclpy
from rclpy.node import Node
import numpy as np
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

def distance(curr_pnt, curr_pos):
    dist = ((curr_pnt.x-curr_pos.x)**2+(curr_pnt.y-curr_pos.y)**2)**0.5
    return dist

def start(avg, curr_pnt, curr_pos):
    dist = distance(curr_pnt, curr_pos)
    print('dist = %d, avg = %d' %(dist, avg))
    brake = 0
    front_steer = 0
    throttle = 15
    if dist < 5.5:
        front_steer = -25
    elif dist < 12:
        front_steer = 25
    elif dist < 30:
        front_steer = 0
        throttle = 0
    else:
        throttle = 0
        if avg <= 25:
            front_steer = -30
        elif avg >= 80:
            front_steer = 10
    return throttle, brake, front_steer
