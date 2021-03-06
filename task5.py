import rclpy
from rclpy.node import Node
import numpy as np
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

def distance(curr_pnt, curr_pos):
    dist = ((curr_pnt.x-curr_pos.x)**2+(curr_pnt.y-curr_pos.y)**2)**0.5
    return dist

def start(avg, now, start, mid, end, curr_pos, pre_pos):
    dist = distance(start, curr_pos)
    dist2 = distance(mid, curr_pos)
    print('dist = %f, dist2 = %f, avg = %d' %(dist, dist2, avg))
    brake = 0
    front_steer = 0
    throttle = 15
    if dist < 10:
        front_steer = 20
    elif dist < 15:
        front_steer = 0
    elif now == mid:
        goal = mid
        dest_vec = np.array([goal.x-curr_pos.x,
                             goal.y-curr_pos.y])
        now_vec = np.array([curr_pos.x - pre_pos.x,
                            curr_pos.y - pre_pos.y])
        dest_vec /= dest_vec.dot(dest_vec)**0.5
        if now_vec.dot(now_vec) == 0:
            now_vec = np.array([0, 0])
        else:
            now_vec /= now_vec.dot(now_vec)**0.5
        dest_rot = np.array([ now_vec[0]*dest_vec[0]+now_vec[1]*dest_vec[1],
                             -now_vec[1]*dest_vec[0]+now_vec[0]*dest_vec[1]])
        angle = -dest_rot[1]
        front_steer = int(90*(angle))
    elif dist2 < 10:
        front_steer = -50
    else:
        goal = end
        dest_vec = np.array([goal.x-curr_pos.x,
                             goal.y-curr_pos.y])
        now_vec = np.array([curr_pos.x - pre_pos.x,
                            curr_pos.y - pre_pos.y])
        dest_vec /= dest_vec.dot(dest_vec)**0.5
        if now_vec.dot(now_vec) == 0:
            now_vec = np.array([0, 0])
        else:
            now_vec /= now_vec.dot(now_vec)**0.5
        dest_rot = np.array([ now_vec[0]*dest_vec[0]+now_vec[1]*dest_vec[1],
                             -now_vec[1]*dest_vec[0]+now_vec[0]*dest_vec[1]])
        angle = -dest_rot[1]
        front_steer = int(100*(angle))
    return throttle, brake, front_steer
