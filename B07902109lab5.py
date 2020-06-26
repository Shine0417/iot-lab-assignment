import rclpy
import math
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

class CheckPoint():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.dis = 10000000
    
    def update(self, x, y):
        dis = (((x - self.x) ** 2) + ((y - self.y) ** 2)) ** 0.5
        if dis < self.dis:
            self.dis = dis

    def getDis(self):
        return self.dis

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)

        self.tmr = self.create_timer(0.01, self.controller_callback)

        self.currentCheckPoint = 0

        self.checkPoints = []

        with open('waypoints.csv') as f:
            line = f.readline()
            while line:
                x, y = [float(v) for v in line.split(',')]
                self.checkPoints.append(CheckPoint(x, y))
                line = f.readline()


        self.old_position = [0, 0];
        self.x = 0
        self.y = 0
        self.steer = 0
    def controller_callback(self):
        msg = RawControlCommand()
        if self.currentCheckPoint == 0:
            msg.throttle = 60
        if self.currentCheckPoint >= 1 and self.currentCheckPoint <= 4:
            msg.throttle = 20
        #elif self.currentCheckPoint >= 7 and self.currentCheckPoint <= 9:
        #    msg.throttle = 5
        elif self.currentCheckPoint >= 17 and self.currentCheckPoint <= 19:
            msg.throttle = 25 
        elif self.currentCheckPoint >= 11 and self.currentCheckPoint <= 14:
            msg.throttle = 30   
        else:
            msg.throttle = 10
        msg.front_steer = self.steer
        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        #print('There are %d bounding boxes.' % len(data.boxes))
        #for box in data.boxes:
        #    print('central of box is at %f %f %f.' % 
        #            (box.centroid.x, box.centroid.y, box.centroid.z))
        #TODO
        return

    def odom_callback(self, data):
        position = data.pose.pose.position
        position.x += 500000

        if math.sqrt((position.x-self.checkPoints[self.currentCheckPoint].x)**2 + (position.y-self.checkPoints[self.currentCheckPoint].y)**2) < 4:
            print ('Waypoint %d passed.' % self.currentCheckPoint)
            self.currentCheckPoint += 1

        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        if(self.old_position[0] == position.x and self.old_position[1] == position.y):
            return
        ov = [0, 0]
        hv = [0, 0]
        ov[0] = position.x-self.old_position[0]
        ov[1] = position.y-self.old_position[1]
        
        hv[0] = self.checkPoints[self.currentCheckPoint].x - position.x 
        hv[1] = self.checkPoints[self.currentCheckPoint].y - position.y 
        
        cos = (ov[0]*hv[0] + ov[1]*hv[1])/math.sqrt((ov[0]*ov[0] + ov[1]*ov[1])*(hv[0]*hv[0] + hv[1]*hv[1]))
        dot = ov[0]*(-hv[1]) + ov[1]*hv[0]

        print('cos :%f dot = %f' %(cos , dot))
        if dot > 0:
            self.steer = int((1.01-cos)* 250)
        elif dot < 0:
            self.steer = -int((1.01-cos)* 250)
        else:
            self.steer = 0
        self.old_position[0] = position.x
        self.old_position[1] = position.y


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


