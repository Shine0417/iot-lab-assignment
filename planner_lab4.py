import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry


class Driver(Node):
    distance = 100
    velocity = 0
    init = 0
    position = 0

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)

        self.tmr = self.create_timer(0.02, self.controller_callback)

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = 0
        msg.brake = 0
        if self.init+13 > self.position:
            if self.init+9 > self.position:
                msg.throttle = 50
            elif self.distance > 6:
                msg.throttle = 10
            else:
                msg.brake = 20
        elif self.init+30 < self.position:
            if self.velocity > 5.0/3.6:
                msg.brake = 50
            else:
                msg.brake = 25
        elif self.velocity > 10.0/3.6:
            msg.brake = 25
        elif self.velocity < 5.0/3.6:
            msg.throttle = 20
        else:
            if self.distance > 18:
                msg.throttle = 50
            elif self.distance > 12:
                msg.throttle = 20
            #elif self.distance > 12:
                #msg.throttle = 0
            elif self.distance > 10:
                msg.brake = 20
            else:
                msg.brake = 50
        #print('throttle = %2f, brake = %2f' %(msg.throttle, msg.brake))
        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        print('There are %d bounding boxes.' % len(data.boxes))
        pre_distance = self.distance
        for box in data.boxes:
        #TODO
            corner_y = [0, 100000]
            corner_x = [0, 100000]
            side = [0.0, 0.0, 1.0, 15.0]
            for corner in box.corners:
                #print('corner of box is %f %f %f.' %
                #        (corner.x, corner.y, corner.z))
                corner_y[0] = max(corner_y[0], corner.y)
                corner_y[1] = min(corner_y[1], corner.y)
                if corner.x > 0:
                    corner_x[0] = max(corner_x[0], corner.x)
                    corner_x[1] = min(corner_x[1], corner.x)
                    for corner2 in box.corners:
                        if (corner == corner2 or 
                            corner.x == corner2.x or 
                            corner.y == corner2.y or
                            #box.size.y > 30 or
                            #box.size.x > 30 or
                            corner.y > 0 and corner2.y > 0 or
                            corner.y < 0 and corner2.y < 0
                            ):
                            continue
                        angle = corner.x**2 + corner.y**2 + corner2.x**2 + corner2.y**2 - (corner.x-corner2.x)**2 - (corner.y-corner2.y)**2
                        angle /= 2* (corner.x**2+corner.y**2)**0.5 * (corner2.x**2+corner2.y**2)**0.5
                        if angle < side[2]:
                            side[0] = corner
                            side[1] = corner2
                            m = (corner.y - corner2.y) / (corner.x - corner2.x)
                            side[3] = (corner.y - m*corner.x) / (m**2 + 1)**0.5
                            if side[3] > 0:
                                self.distance = side[3]
            #distance = side[3]
            distance = corner_x[0]
            if (corner_y[0] > 0 and corner_y[1] < 0 and corner_x[0] > 0 and corner_x[1] < 0):
                self.distance = 0
                print('size of box is %f %f %f.' %(box.size.x, box.size.y, box.size.z))
            velocity = box.velocity

            isFront = (box.centroid.y+box.size.y/2 > 0 and box.centroid.y-box.size.y/2 < 0
                      # and box.centroid.z+box.size.z/2 > 0 and box.centroid.z-box.size.z/2 < 0
                      and corner_x[0] > 0
                      #and box.centroid.x > 0
                      and distance>=0
                      and box.size.y < 20
                      and box.size.x < 15
                      and box.orientation.z > -0.9
                      and corner_y[0] > 0 and corner_y[1] < 0
                      )
            if isFront:
                #print('central of box is at %f %f %f.' % 
                #        (box.centroid.x, box.centroid.y, box.centroid.z))
                self.distance = distance
                #print('heading of box is %f.' %box.heading)
                #print('heading rate of box is %f.' %box.heading_rate)
                #print('velocity of box is %f.' %velocity)
                #print('size of box is %f.' %box.size.y)
                #print('orientation of box is %f.' %box.orientation.z)
        if self.distance == pre_distance:
            self.distance = 14
        self.pre_distance = pre_distance
        print('distance = %f.' %self.distance)

    def odom_callback(self, data):
        position = data.pose.pose.position
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #TODO
        vector = data.twist.twist.linear
        self.velocity = (vector.x**2+vector.y**2+vector.z**2)**0.5
        #print('velocity = %f.' %self.velocity)
        if self.init == 0:
            self.init = position.x
        else:
            self.position = position.x


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


