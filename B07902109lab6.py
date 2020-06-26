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
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)
        self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
                self.camera_callback, 10)

        self.tmr = self.create_timer(0.5, self.controller_callback)
        
        self.front_steer= 0
        self.brake = 0

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = 10
        msg.front_steer = int(self.front_steer)
        msg.brake = int(self.brake)
        self.pub.publish(msg)

    def odom_callback(self, data):
        position = data.pose.pose.position
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #TODO


    def camera_callback(self, data):
        if self.front_steer > 0:
            self.front_steer = 0

        RGB_image = cv2.cvtColor(cv2.imdecode(np.array(data.data, dtype=np.uint8), cv2.IMREAD_COLOR) , cv2.COLOR_BGR2RGB)
        
        view = bird(RGB_image)
        
        rightx, righty = find_road(view)

        if (max(rightx)-min(rightx)) >= 50:
            s = 0
            cnt = 0
            fit = np.polyfit(rightx, righty, 2)

            for x in rightx:
                s = s + formula(x, fit)
                cnt = cnt + 1

            avg = s/(cnt*100)

            print(f"curve detected! avg radius = {int(avg)}, curve = {1/avg}")

            if avg <= 25:
                self.front_steer = -32
                self.brake = 10
            elif avg >= 80:
                self.front_steer = 10
                self.brake = 0
            else:
                self.front_steer = 0
                self.brake = 0
            
rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


