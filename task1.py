import rclpy
from rclpy.node import Node


from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import matplotlib.pyplot as plt
import cv2
import numpy as np

def bird(image):
    blue_image = image[:,:,2]
    cv2.imwrite("blue.jpeg", blue_image)#Somehow helps....

    source = np.float32([[888, 603], [675.5, 773], [1115, 773], [1013, 603]])

    trans = np.float32([[56, 0], [56, 1984], [344, 1984], [344, 0]])

    ratio = cv2.getPerspectiveTransform(source, trans)

    bird_view = cv2.warpPerspective(blue_image, ratio, (500, 2200))
    cv2.imwrite("bird_view.jpeg", bird_view)
    
    return bird_view[:,:]

def find_road(view):

    road = np.zeros_like(view)
    road[(view <= 150) & (view >= 80)] = 1
    cv2.imwrite("road.jpeg", road)

    hist = np.sum(road[road.shape[0]//2:,:], axis=0)
    midpoint = hist.shape[0]//2

    rightx = []
    righty = []

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

    return rightx, righty

def formula(x, fit):

    return ((1 + (2*fit[0]*x + fit[1])**2)**1.5) / np.absolute(2*fit[0])

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
        hsv = cv2.cvtColor(image , cv2.COLOR_BGR2HSV)

        # color range
        lower_red1 = np.array([0,100,100])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([160,100,100])
        upper_red2 = np.array([180,255,255])
        
        lower_green = np.array([40,50,50])
        upper_green = np.array([90,255,255])

        lower_yellow = np.array([15,150,150])
        upper_yellow = np.array([35,255,255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maskg = cv2.inRange(hsv, lower_green, upper_green)
        masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
        maskr = cv2.add(mask1, mask2)

        
        # determine the bottom of line bases according to lower quarter of BCB
        size = image.shape
        # print size

        # hough circle detect
        r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 30,
                                   param1=50, param2=10, minRadius=0, maxRadius=30)

        y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, 30,
                                     param1=50, param2=10, minRadius=0, maxRadius=30)

        # traffic light detect
        r = 5
        bound = 0.5
        red_or_yellow = 0
        if r_circles is not None:
            r_circles = np.uint16(np.around(r_circles))

            for i in r_circles[0, :]:
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
                    red_or_yellow = 1
                #print("h/s=",h/s)

        if y_circles is not None:
            y_circles = np.uint16(np.around(y_circles))

            for i in y_circles[0, :]:
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
                    red_or_yellow = 1
                # print("yh/s=",h/s)
        if red_or_yellow == 0:
            self.throttle = 10
            self.brake = 0

        #print(f"red or yellow {red_or_yellow}")
        
        
        if self.steer != 0:
            self.steer = 0
        RGB_image = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
        
        view = bird(RGB_image)
        
        rightx, righty = find_road(view)
        # decide whether the car is parallel to the road
        sum = 0
        for i in range(len(rightx)-1):
            sum  = sum + rightx[i+1] - rightx[i]
        if sum <= -5 and sum >= -10:
            self.steer = 1
        elif sum >= 4 and sum <= 10:
            self.steer = -1
        else:
            self.steer = 0
        #print(self.steer)

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


