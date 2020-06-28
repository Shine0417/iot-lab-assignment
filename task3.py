import rclpy
from rclpy.node import Node
import numpy as np
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

def distance(curr_pnt, curr_pos):
    dist = ((curr_pnt.x-curr_pos.x)**2+(curr_pnt.y-curr_pos.y)**2)**0.5
    return dist

def start(avg, curr_pnt, curr_pos, rightx, righty):
    dist = distance(curr_pnt, curr_pos)
    print('dist = %d, avg = %d' %(dist, avg))
    brake = 0
    front_steer = 0
    throttle = 15
    # if dist < 5.5:
    #     front_steer = -25
    # elif dist < 12:
    #     front_steer = 28
    # elif dist < 35:
    #     front_steer = 1
    #     throttle = 0
    # elif dist < 45:
    #     front_steer = 0
    #     throttle = 0
    if dist < 36:
        front_steer = 0
    elif dist < 43:
        front_steer = -25
    elif dist < 50:
        front_steer = 25
    else:
        throttle = 1
        front_steer = 0
        if avg <= 15:
            front_steer = -40
        elif avg >= 80:
            front_steer = 10
    return throttle, brake, front_steer
