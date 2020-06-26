import rclpy
from rclpy.node import Node
import numpy as np
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry
import task1
import task2
import task3
import task4
import task5
import task6

DIS_THROSHOLD = 3

class CheckPoint():
    def __init__(self, x, y, task):
        self.x = x
        self.y = y
        self.task = task
        self.dis = 10000000

    def update(self, x, y):
        dis = (((x - self.x) **2) + ((y - self.y)**2)) **0.5
        if dis < self.dis:
            self.dis = dis

    def getDis(self):
        return self.dis

checkPoints = []
taskToCheckPoints = {}
with open('waypoints_2.csv') as f:
    line = f.readline()
    task = 0
    while line:
        points = [float(v) for v in line.split(',')]
        if len(points) == 2:
            x, y = points
            if x == 64.5 and y == -52.0:
                task = 0
            elif x == -9.2 and y == 54.3:
                task = 0
        else:
            x, y, task = points
            taskToCheckPoints[task] = len(checkPoints)
        checkPoints.append(CheckPoint(x, y, task))
        line = f.readline()
for checkPoint in checkPoints:
    print('x = %d, y = %d, task = %d' %(checkPoint.x, checkPoint.y, checkPoint.task))

class Driver(Node):
    currentCheckPoint = 0
    position = None
    pre_position = None
    velocity = None
    currentTask = 0  # You can change the task to test
    if currentTask:
        currentCheckPoint = taskToCheckPoints[currentTask]

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)

        self.tmr = self.create_timer(0.05, self.controller_callback)

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = 30
        msg.front_steer = 0
        if self.position and self.pre_position:
            # Speed limit to 30 m/s
            if ((self.velocity.x**2)+(self.velocity.y**2)+(self.velocity.z**2))**0.5 > 30.0/3.6:
                msg.throttle = 0

            if self.currentTask == 0: # Direct by waypoints
                dest_vec = np.array([checkPoints[self.currentCheckPoint].x-self.position.x,
                                     checkPoints[self.currentCheckPoint].y-self.position.y])
                now_vec = np.array([self.position.x - self.pre_position.x,
                                    self.position.y - self.pre_position.y])
                dest_vec /= dest_vec.dot(dest_vec)**0.5
                if now_vec.dot(now_vec) == 0:
                    now_vec = np.array([0, 0])
                else:
                    now_vec /= now_vec.dot(now_vec)**0.5
                dest_rot = np.array([ now_vec[0]*dest_vec[0]+now_vec[1]*dest_vec[1],
                                     -now_vec[1]*dest_vec[0]+now_vec[0]*dest_vec[1]])
                angle = -dest_rot[1]
                '''
                print('waypoint = %d' %(self.currentCheckPoint))
                print('dest = %f %f' %(dest_vec[0], dest_vec[1]))
                print('now = %f %f' %(now_vec[0], now_vec[1]))
                print('angle = %f' %(angle))
                '''
                msg.front_steer = int(100*(angle))
            elif self.currentTask == 1: # Traffic Light
                msg.throttle, msg.brake, msg.front_steer = task1.start()
            elif self.currentTask == 2: # Road Curve
                msg.throttle, msg.brake, msg.front_steer = task2.start()
            elif self.currentTask == 3: # Narrowing Driving Lanes
                msg.throttle, msg.brake, msg.front_steer = task3.start()
            elif self.currentTask == 4: # T-junction
                msg.throttle, msg.brake, msg.front_steer = task4.start()
            elif self.currentTask == 5: # Roundabout
                msg.throttle, msg.brake, msg.front_steer = task5.start()
            elif self.currentTask == 6: # Angled Parking Slots
                msg.throttle, msg.brake, msg.front_steer = task6.start()

        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        '''
        print('There are %d bounding boxes.' % len(data.boxes))
        for box in data.boxes:
            print('central of box is at %f %f %f.' % 
                    (box.centroid.x, box.centroid.y, box.centroid.z))
        '''
        #TODO

    def odom_callback(self, data):
        position = data.pose.pose.position
        if position.x < -250000:
            position.x = position.x + 500000
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        velocity = data.twist.twist.linear
        #print('Current vel: %f, %f, %f' % (velocity.x, velocity.y, velocity.z))
        #TODO
        self.pre_position = self.position
        self.position = position
        self.velocity = velocity
        for i in range(-2, 1):
            if self.currentCheckPoint + i >= 0 and self.currentCheckPoint + i < len(checkPoints):
                checkPoints[self.currentCheckPoint+i].update(self.position.x, self.position.y)
        if self.currentCheckPoint < len(checkPoints) and checkPoints[self.currentCheckPoint].getDis() < DIS_THROSHOLD:
            #print('x = %d, y = %d, task = %d' %(checkPoints[self.currentCheckPoint].x, checkPoints[self.currentCheckPoint].y, checkPoints[self.currentCheckPoint].task))
            print('Waypoint %d passed, Task %d.' % (self.currentCheckPoint, checkPoints[self.currentCheckPoint].task))
            self.currentCheckPoint += 1
            self.currentTask = checkPoints[self.currentCheckPoint].task


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


