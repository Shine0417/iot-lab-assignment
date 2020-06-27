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

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        # self.create_subscription(Odometry, '/lgsvl_odom',
        #         self.odom_callback, 10)
        self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
                self.camera_callback, 10)

        self.tmr = self.create_timer(0.1, self.controller_callback)

        self.steer= 0

        self.throttle = 0
        
        self.brake = 0

        self.right_line = None

        self.left_line = None

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = int(self.throttle)
        msg.front_steer = int(self.steer)
        msg.brake = int(self.brake)
        self.pub.publish(msg)

    def camera_callback(self, data):
        mpl.use('TkAgg')
        bird, BCB = self.image_tranform(data, cv2.COLOR_BGR2RGB, 2, 100, 190)

        a, BCB2 = self.image_tranform(data, cv2.COLOR_BGR2LUV, 0, 130, 190)
        b, BCB3 = self.image_tranform(data, cv2.COLOR_BGR2LUV, 1, 110, 190)

        BCB4 = np.zeros_like(BCB2)
        BCB4[(BCB2 == 1) & (BCB3 == 1)] = 1
        # plt.imshow(BCB4)
        # plt.show()

        right_line = self.draw_line(40 , 15 , 20 , BCB , 1)
        left_line = self.draw_line(80 , 15 , 60 , BCB4 , 0)

        right_x , right_y = self.line_to_go(100 , right_line , 1)
        left_x , left_y = self.line_to_go(230 , left_line , 0)

        weight_R = np.zeros_like(right_x)
        weight_R[:weight_R.shape[0]//6] = 20
        weight_R[weight_R.shape[0]//6:] = 1
        #print(weight_Rx)
        weight_L = np.zeros_like(left_x)
        weight_L[:min(3, weight_L.shape[0]//10 + 1)] = 40
        weight_L[min(3, weight_L.shape[0]//10 + 1):] = 1
        #print(weight_L)
        #print(left_x)

        if len(left_x) == 0 and len(right_x) == 0:
            x = 1000
            y = 4900
            print("fuck")
        elif len(left_x) == 0:
            print("determine by right line")
            x = np.average(right_x, weights = weight_R)
            y = np.average(right_y, weights = weight_R)
        elif len(right_x) == 0:
            x = np.average(left_x, weights = weight_L)
            y = np.average(left_y, weights = weight_L)
        else:
            x = (np.average(left_x, weights = weight_L) * 20 + np.average(right_x, weights = weight_R)) / 21
            y = (np.average(left_y, weights = weight_L) * 20 + np.average(right_y, weights = weight_R)) / 21

        # print("x=", x)
        # print("y=", y)

        if sys.argv[1] == "show":
            plt.imshow(bird)
            #plt.imshow(BCB)
            for i in range(len(right_x) - 1):
                plt.plot([right_x[i], right_x[i+1]], [right_y[i], right_y[i+1]], color="red")
            for i in range(len(right_line) - 1):
                plt.plot([right_line[i][0], right_line[i+1][0]], [right_line[i][1], right_line[i+1][1]], color="red")
            plt.plot([1000, np.average(x)], [5000, np.average(y)], color="yellow")
            # plt.show()

            #plt.imshow(BCB4)
            for i in range(len(left_x) - 1):
                plt.plot([left_x[i], left_x[i+1]], [left_y[i], left_y[i+1]], color="green")
            for i in range(len(left_line) - 1):
                plt.plot([left_line[i][0], left_line[i+1][0]], [left_line[i][1], left_line[i+1][1]], color="red")
            plt.plot([1000, np.average(x)], [5000, np.average(y)], color="yellow")
            plt.show()

        direction = np.array([0, -100])
        target = np.array([x-1000, y-5000])
        #print(np.average(right_x)+np.average(right_y))
        #print(target);
        length_d = np.sqrt(direction.dot(direction))
        length_t = np.sqrt(target.dot(target))
        theta = np.arcsin(np.cross(direction, target)/(length_t * length_d)) 
        self.steer = int(theta * 100 / np.pi)
        self.throttle = 3
        # self.throttle = 8
        # if(abs(self.steer) >= 15):
        #     self.throttle = int(16 * abs(self.steer) / 18)
        # else:
        #     self.brake = 3

        # if self.steer >= 30 or self.steer <= -30: self.throttle = 15
        # elif self.steer >= 10 or self.steer <= -10: self.brake = 5
        # else: self.brake = 0

        print("theta =", theta)
        print("throttle =", self.throttle)
        print("brake =", self.brake)
        print("steer =", self.steer)
        print("========================")

    def image_tranform(self, data, ch, color, thr_min, thr_max):
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image , ch)
        image = image[:,:,color]
        # cv2.imwrite("output.jpeg", image)
        # plt.imshow(image)
        # plt.show()
        # convert the image to aerial view according to specific tested points
        src = np.float32([[888, 603.3], [675.5, 773.3], [1115, 773.3], [1013, 603.3]])
        # dst = np.float32([[56, 0], [56, 1984], [344, 1984], [344, 0]])
        dst = np.float32([[850, 0], [850, 5000], [1150, 5000], [1150, 0]])
        M = cv2.getPerspectiveTransform(src, dst)
        aerial_view = cv2.warpPerspective(image, M, (2000, 5000))
        # cv2.imwrite("rec.jpeg", aerial_view)
        # plt.imshow(aerial_view)
        # plt.show()

        # BCB representing blue channel binary, and we make pixel 1 if its brightness is between threshold
        channel = aerial_view[:,:]
        threshold = (thr_min, thr_max)
        BCB = np.zeros_like(channel)
        BCB[(channel >= threshold[0]) & (channel <= threshold[1])] = 1
        # cv2.imwrite("BCB.jpeg", BCB)
        # plt.imshow(BCB)
        # plt.show()
        return aerial_view, BCB

    def draw_line(self , block_h  , lane_range , block_count , BCB , side):
        # determine the bottom of line bases according to lower quarter of BCB
        # block_h 50 lane_range 70 block_count 20
        histogram = None
        if side == 0:
            histogram = np.sum(BCB[int(BCB.shape[0] * 0.3):,:], axis=0)
        else:
            histogram = np.sum(BCB[int(BCB.shape[0] * 0.75):,:], axis=0)

        midpoint = histogram.shape[0]//2
        # for i in range(histogram.shape[0]):
        #     histogram[i] = int(histogram[i] * 1.2 * (((midpoint - abs(i - midpoint)) / midpoint)) )
            #print(i, histogram[i] * (((midpoint - abs(i - midpoint)) / midpoint) + 0.1))
        lefts_base = np.argmax(histogram[:])
        rights_base = np.argmax(histogram[midpoint:]) + midpoint
        if side == 0:
            prev_base = lefts_base
        else:
            prev_base = rights_base

        if side == 1 and np.argmin(histogram[midpoint:]) + midpoint == rights_base:
            prev_base = 320

        #print(side, rights_base)

        points = []

        diff = 0

        for i in range(1, block_count):
            #print(diff)
            block = np.sum(BCB[BCB.shape[0] - block_h * i : BCB.shape[0] - block_h * (i-1), :], axis = 0)
            #print("before:", block)
            for j in range(max(0, prev_base - lane_range), min(prev_base + lane_range, BCB.shape[1])):
                block[j] = int(block[j] * 1.5 * (-1*np.sign(j-prev_base)*max((j-prev_base), 1)/lane_range + 2))
            #print("after:", block)
            if prev_base - lane_range >= BCB.shape[1] or prev_base + lane_range <= 0:
                break;
            else:
                blank_check = np.argmin(block[max(0, prev_base - lane_range): min(prev_base + lane_range, BCB.shape[1])]) + max(0, prev_base - lane_range)
                line = np.argmax(block[max(0, prev_base - lane_range): min(prev_base + lane_range, BCB.shape[1])]) + max(0, prev_base - lane_range)
                # break if line touchs boundary
                if line <= 0 or line >= BCB.shape[1]:
                    break
                # if there's no line in block, then create a virtual line according to base
                if blank_check == line:
                    #print("no detecting road line")
                    line = prev_base + diff
                elif side == 1 or blank_check != line:
                    points.append([line, BCB.shape[0] - block_h * (i-1) - block_h / 2])
                # update base 
                diff = line - prev_base
                prev_base = line
        return points

    def line_to_go(self , distance , points , side):
        vec = []
        mid = []
        x = []
        y = []
        waypoints = []
        for idx in range(1, len(points)):
            #vec.append(np.array[points[idx][0]-points[idx-1][0], points[idx][1]-points[idx-1][1]])
            #mid.append(np.array[(points[idx][0]+points[idx-1][0])/2, (points[idx][1]+points[idx-1][1])/2])
            if side == 0:
                vertical_vec = np.array([-1*(points[idx][1]-points[idx-1][1]), (points[idx][0]-points[idx-1][0])])
            else:
                vertical_vec = np.array([(points[idx][1]-points[idx-1][1]), -1*(points[idx][0]-points[idx-1][0])])
            vertical_vec = vertical_vec / (vertical_vec[0] ** 2 + vertical_vec[1] ** 2) ** (1/2) * distance
            waypoints.append([(points[idx][0]+points[idx-1][0])/2+vertical_vec[0], (points[idx][1]+points[idx-1][1])/2+vertical_vec[1]])
            x.append(waypoints[idx-1][0])
            y.append(waypoints[idx-1][1])

            p1 = [(points[idx][0]+points[idx-1][0])/2, (points[idx][1]+points[idx-1][1])/2]
            #plt.plot([p1[0], p1[0]+vertical_vec[0]], [p1[1], p1[1]+vertical_vec[1]], color="green")
            #plt.plot([points[idx][0], points[idx-1][0]], [points[idx][1], points[idx-1][1]], color="blue")

        # find the fitting line to the points
        #fit = np.polyfit(x, y, 2)
        x = np.array(x)
        y = np.array(y)
        return x , y
        

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


