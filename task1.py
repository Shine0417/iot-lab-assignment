import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import matplotlib.pyplot as plt
import cv2
import numpy as np


class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        # self.create_subscription(Odometry, '/lgsvl_odom',
        #         self.odom_callback, 10)
        self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
                self.camera_callback, 10)

        self.tmr = self.create_timer(0.5, self.controller_callback)
        self.steer= 0
        self.throttle = 0
        self.brake = 0

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = int(self.throttle)
        msg.front_steer = int(self.steer)
        msg.brake = int(self.brake)
        self.pub.publish(msg)

    def camera_callback(self, data):

        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        # reference from https://github.com/HevLfreis/TrafficLight-Detector/blob/master/src/main.py
        # color range
        thresh_red_low1 = np.array([0,100,100])
        thresh_red_high1 = np.array([10,255,255])
        thresh_red_low2 = np.array([160,100,100])
        thresh_red_high2 = np.array([180,255,255])
        thresh_yellow_low = np.array([15,150,150])
        thresh_yellow_high = np.array([35,255,255])

        hsv = cv2.cvtColor(image , cv2.COLOR_BGR2HSV)
        r1 = cv2.inRange(hsv, thresh_red_low1, thresh_red_high1)
        r2 = cv2.inRange(hsv, thresh_red_low2, thresh_red_high2)
        # maskg = cv2.inRange(hsv, lower_green, upper_green)
        y_thresh = cv2.inRange(hsv, thresh_yellow_low, thresh_yellow_high)
        r_thresh = cv2.add(r1, r2)

        # hough circle detect
        red = cv2.HoughCircles(r_thresh, cv2.HOUGH_GRADIENT, 1, 30,
                                   param1=50, param2=10, minRadius=0, maxRadius=30)
        yellow = cv2.HoughCircles(y_thresh, cv2.HOUGH_GRADIENT, 1, 30,
                                     param1=50, param2=10, minRadius=0, maxRadius=30)

        # traffic light detect
        r = 5
        bound = 0.5
        stop = 0
        size = image.shape
        # red light detected
        if red is not None:
            red = np.uint16(np.around(red))
            for i in red[0, :]:
                # print(i)
                # print(size)
                if i[1] > size[0] or i[1] > size[0] * bound:
                    continue
                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            print(i[1]+m , i[0]+n)
                            continue
                        h += maskr[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 200:
                    self.throttle = 0
                    self.brake = 20
                    stop = 1
                #print("h/s=",h/s)
        # yellow light detect
        if yellow is not None:
            yellow = np.uint16(np.around(yellow))
            for i in yellow[0, :]:
                if i[1] > size[0] or i[1] > size[0]*bound:
                    continue
                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += masky[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 200:
                    self.throttle = 0
                    self.brake = 20
                    stop = 1
                # print("yh/s=",h/s)
        # green light
        if stop == 0:
            self.throttle = 10
            self.brake = 0

        # prevent lag
        if self.steer != 0:
            self.steer = 0

        image = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
        blue_image = image[:,:,2]
        #Somehow helps....
        cv2.imwrite("blue.jpeg", blue_image)
        #turn road into bird eye view
        source = np.float32([[888, 603], [675.5, 773], [1115, 773], [1013, 603]])

        trans = np.float32([[56, 0], [56, 1984], [344, 1984], [344, 0]])

        ratio = cv2.getPerspectiveTransform(source, trans)

        view = cv2.warpPerspective(blue_image, ratio, (500, 2200))

        road = np.zeros_like(view)
        # threshold = 80~150
        road[(view <= 150) & (view >= 80)] = 1
        cv2.imwrite("road.jpeg", road)

        hist = np.sum(road[road.shape[0]//2:,:], axis=0)
        midpoint = hist.shape[0]//2

        rightx = []
        righty = []
        # find riht line
        prev = np.argmax(hist[midpoint:]) + midpoint
        for i in range(1, 50):
            his = np.sum(road[road.shape[0]-9*i:road.shape[0]-9*(i-1),:], axis = 0)

            thresh = (prev-20, prev+20)
            now = np.argmax(his[max(0, thresh[0]):min(thresh[1],road.shape[1])]) + max(0,thresh[0])
            
            if now >= road.shape[1]:
                break
            elif now <= 0:
                break
            elif now != np.argmin(his[max(0, thresh[0]):min(thresh[1],road.shape[1])]) + max(0,thresh[0]):
                prev = now

            rightx.append(prev)
            righty.append(road.shape[0]-9*(i-1)-9/2)

        # decide whether the car is parallel to the road
        offset = 0
        self.steer = 0
        for i in range(len(rightx)-1):
            offset = offset + rightx[i+1] - rightx[i]
        if offset >= 4 and offset <= 10:
            self.steer = -1
        if offset <= -5 and offset >= -10:
            self.steer = 1

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


