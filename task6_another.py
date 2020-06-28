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
# def bird(image):
#     blue_image = image[:,:,2]
#     cv2.imwrite("blue.jpeg", blue_image)#Somehow helps....

#     source = np.float32([[888, 603], [675.5, 773], [1115, 773], [1013, 603]])

#     trans = np.float32([[56, 0], [56, 1984], [344, 1984], [344, 0]])

#     ratio = cv2.getPerspectiveTransform(source, trans)

#     bird_view = cv2.warpPerspective(blue_image, ratio, (500, 2200))
#     cv2.imwrite("bird_view.jpeg", bird_view)
    
#     return bird_view[:,:]


class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)

        # self.create_subscription(Odometry, '/lgsvl_odom',
        #         self.odom_callback, 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes',
                self.bounding_boxes_callback, 10)
        # self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
        #         self.camera_callback, 10)

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
            if abs(box.size.x - box.size.y) < 1 and box.size.x < 3.2 and box.size.x > 2:
                self.steer = 13
                print('central  %0.3f %0.3f %0.3f, size %0.3f %0.3f %0.3f.' %(box.centroid.x, box.centroid.y, box.centroid.z, box.size.x, box.size.y, box.size.z))
                if box.centroid.x < 2.5:
                    self.throttle = 0
                    self.brake = 75

                # if box.centroid.y < 0:
                #     self.steer = 10
                # if box.centroid.y > 0:
                #     self.steer = -5

            # print('size of box is at %f %f %f.' %(box.size.x, box.size.y, box.size.z))
            # if box.centroid.x >= 0 and box.centroid.y < 0 and box.centroid.y > -0.6 and abs(box.centroid.z) < 1:
            #     if abs(box.centroid.x) < 20:       
            #         self.stop = 1;
            #         print('Stop box: %f, %f, %f' % (box.centroid.x, box.centroid.y, box.centroid.z))
            #         break;
    # def odom_callback(self, data):
    #     position = data.pose.pose.position
    #     position.x += 500000

    # def camera_callback(self, data):
        # mpl.use('TkAgg')
        # image_data = np.array(data.data, dtype=np.uint8)
        # image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # # plt.imshow(image)
        # # plt.show()
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # # plt.imshow(image)
        # # plt.show()
        # src = np.float32([[888, 603.3], [675.5, 773.3], [1115, 773.3], [1013, 603.3]])
        # dst = np.float32([[56, 0], [56, 1984], [344, 1984], [344, 0]])
        # M = cv2.getPerspectiveTransform(src, dst)
        # image = cv2.warpPerspective(image, M, (500, 2200))
        
        # kernel_size = 5
        # blur_gray = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

        # low_thresh = 50
        # high_thresh = 150
        # edges = cv2.Canny(blur_gray, low_thresh, high_thresh)
        # cv2.imwrite('edges.jpeg', edges)

        # plt.imshow(edges)

    #     # mpl.use('TkAgg')
    #     image_data = np.array(data.data, dtype=np.uint8)
    #     image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    #     cv2.imwrite('image.jpeg',image)

    
    #     RGB_image = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
        
    #     view = bird(RGB_image)
        
    #     road = np.zeros_like(view)
    #     road[(view <= 150) & (view >= 80)] = 1
    #     cv2.imwrite('road.jpeg',road)
    

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()
