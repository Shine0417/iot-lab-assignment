import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry


class Driver(Node):
    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 50)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 50)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)

        self.tmr = self.create_timer(0.25, self.controller_callback)

        self.accelerate = 0
        self.last_dist = 5
        self.dist = 5
    def controller_callback(self):
        #print(self.accelerate)
        msg = RawControlCommand()
        a = (self.dist - self.last_dist)*10
        print('dist: %d, a: %d' %(self.dist, a))
        if self.dist >= 5 and self.dist <= 7 and a > -10:
            msg.throttle = 50
            msg.brake = 0
        elif self.dist + a < 5:
            msg.throttle = 0
            msg.brake = 100
            print('brake')
        elif self.dist > 7:
            msg.throttle = 60
            msg.brake = 0
        else:
            msg.throttle = 0
            msg.brake = 40
            print('else brake')
        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        #print('There are %d bounding boxes.' % len(data.boxes))
        
        for box in data.boxes:
            #print('central of box is at %f %f %f.' % (box.centroid.x, box.centroid.y, box.centroid.z))
            if box.centroid.x >= 0 and box.centroid.x < 25 and box.centroid.y < -0.1 and box.centroid.y > -0.5 and box.centroid.z < 1 and box.centroid.z > 0.3:       
                self.last_dist = self.dist;
                self.dist = box.centroid.x;
                #print('detected')
            
        #TODO

    def odom_callback(self, data):
        position = data.pose.pose.position
        self.accelerate = data.twist.twist.linear.x
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #print('%f' %(data.pose.pose.covariance))
        #TODO


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


