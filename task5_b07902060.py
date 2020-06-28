# your code goes here
import rclpy
from rclpy.node import Node
import numpy as np


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry



class Driver(Node):
    
    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)

        self.tmr = self.create_timer(0.01, self.controller_callback)
        self.checkPoints = [[36.2, -61.9], [35.2, -35.9]]
        self.steer = 0
        self.throttle = 0
        self.distance = None
        self.check = 0
        self.prev = None
        self.direction = None
        self.target = None
        self.count = 0
        self.t_count = 280

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = self.throttle
        msg.front_steer = self.steer
        self.pub.publish(msg)

    def odom_callback(self, data):
        if self.t_count >= 0 and self.check == 0:
            self.throttle = 3
            self.steer = 20
            self.t_count -= 1
            print("t_count =", self.t_count)
            return
        elif self.t_count >= 0 and self.check == 1:
            self.throttle = 3
            self.steer = -45
            self.t_count -= 1
            print("t_count =", self.t_count)
            return

        position = data.pose.pose.position
        position.x += 500000
        #print('Current pos: %f, %f' % (position.x, position.y))
        #print('waypoint: %f, %f' % (self.checkPoints[self.check][0], self.checkPoints[self.check][1]))
        if self.prev != None:
            if self.prev[0] != position.x or self.prev[1] != position.y:
                self.direction = np.array([(position.x - self.prev[0]), (position.y - self.prev[1])])
                self.target = np.array([self.checkPoints[self.check][0] - position.x, self.checkPoints[self.check][1] - position.y])
                La = np.sqrt(self.direction.dot(self.direction))
                Lb = np.sqrt(self.target.dot(self.target))
                theta = np.arcsin(np.cross(self.direction , self.target) / (La * Lb))
                self.steer = -int(theta * 275 / np.pi)
                self.distance = self.dis([position.x, position.y], self.checkPoints[self.check])
                if self.distance < 10:
                    self.throttle = 3
                else:
                    self.throttle = 10
                if self.distance < 2.5:
                    self.check += 1
                    self.t_count = 120
        self.prev = [position.x, position.y]
        self.count += 1
        print("steer =", self.steer)
        #TODO
    def dis(self, a, b):
        return (((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


