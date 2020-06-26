import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2, math

# Define a class to handle all help functions
class Helper():
    def __init__(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Global Variable: image size in px as pair (height, width)
        self.image_size = image.shape[:2]
        # Global Variable: image height in px
        self.image_height = self.image_size[0]
        # Global Variable: image width in px
        self.image_width = image.shape[1]
        # Global Variable: camera transformation matrix
        self.mtx = None
        # Global Variable: inverse camera transformation matrix
        self.inverse_matrix = None
        # Global Variable: camera transformation matrix
        self.dist = None
        ## Global Variable: source transformation matrix
        #self.src = np.float32([(920, 600), (1070, 600), (830, 700), (1160, 700)])
        self.src = np.float32([(820, 700), (1155, 700), (730, 800),(1270, 800)])
        ## Global Variable: destination transformation matrix
        self.dst = dst = np.float32([(450, 0), (self.image_width - 450,0), (450, self.image_height), (self.image_width - 450, self.image_height)])

    def unwarp(self, img):
        # Calculate transformation matrix
        matrix = cv2.getPerspectiveTransform(self.src, self.dst)
        inverse_matrix = cv2.getPerspectiveTransform(self.dst, self.src)
        # record inverse matrix
        self.inverse_matrix = inverse_matrix
        # Apply transformation
        warped = cv2.warpPerspective(img, matrix, (self.image_width, self.image_height), flags=cv2.INTER_LINEAR)
        return warped

    # Define a function that thresholds the L-channel of HLS
    # Use exclusive lower bound (>) and inclusive upper (<=)
    def hls_lthresh(self, img, thresh=(180, 255)):
        # 1) Convert to HLS color space
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        hls_l = hls[:,:,1]
        if np.max(hls_l) > 85:
            hls_l = hls_l*(255/np.max(hls_l))
        # 2) Apply a threshold to the L channel
        binary_output = np.zeros_like(hls_l)
        binary_output[(hls_l > thresh[0]) & (hls_l <= thresh[1])] = 1
        # 3) Return a binary image of threshold result
        return binary_output

    # Define a function that thresholds the B-channel of LAB
    # Use exclusive lower bound (>) and inclusive upper (<=), OR the results of the thresholds (B channel should capture
    # yellows)
    def lab_bthresh(self, img, thresh=(180,255)):
        # 1) Convert to LAB color space
        lab = cv2.cvtColor(img, cv2.COLOR_RGB2Lab)
        lab_b = lab[:,:,2]
        lab_l = lab[:,:,0]
        #lab_a = lab[:,:,1]
        # don't normalize if there are no yellows in the image
        if np.max(lab_b) > 185:
            lab_b = lab_b*(255/np.max(lab_b))
        if np.max(lab_l) > 145:
            lab_l = lab_l*(255/np.max(lab_l))
        '''
        if np.max(lab_a) > 185:
            lab_a = lab_a*(255/np.max(lab_a))
        '''
        # 2) Apply a threshold to the L channel
        binary_output = np.zeros_like(lab_b)
        binary_output[((lab_l < 190) & 
                        #(lab_a > 150) &
                        (lab_b > thresh[0]) & 
                        (lab_b <= thresh[1]))] = 1
        # 3) Return a binary image of threshold result
        return binary_output

    def calc_curvature_and_center_dist(self, bin_img, l_fit, r_fit, l_lane_inds, r_lane_inds):
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 3.048/100 # meters per pixel in y dimension, lane line is 10 ft = 3.048 meters
        xm_per_pix = 3.7/378 # meters per pixel in x dimension, lane width is 12 ft = 3.7 meters
        left_curverad, right_curverad, center_dist = (0, 0, 0)
        # Define y-value where we want radius of curvature
        # I'll choose the maximum y-value, corresponding to the bottom of the image
        h = bin_img.shape[0]
        ploty = np.linspace(0, h-1, h)
        y_eval = np.max(ploty)
    
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = bin_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Again, extract left and right line pixel positions
        leftx = nonzerox[l_lane_inds]
        lefty = nonzeroy[l_lane_inds] 
        rightx = nonzerox[r_lane_inds]
        righty = nonzeroy[r_lane_inds]
        
        if len(leftx) != 0 and len(rightx) != 0:
            # Fit new polynomials to x,y in world space
            left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
            right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
            # Calculate the new radii of curvature
            left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
            right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
            # Now our radius of curvature is in meters
        
        # Distance from center is image x midpoint - mean of l_fit and r_fit intercepts 
        if r_fit is not None and l_fit is not None:
            car_position = bin_img.shape[1]/2
            l_fit_x_int = l_fit[0]*h**2 + l_fit[1]*h + l_fit[2]
            r_fit_x_int = r_fit[0]*h**2 + r_fit[1]*h + r_fit[2]
            lane_center_position = (r_fit_x_int + l_fit_x_int) /2
            center_dist = (car_position - lane_center_position) * xm_per_pix
        return round(left_curverad, 1), round(right_curverad, 1), round(center_dist, 1)

    # Image pre processing pipeline
    def pre_pipeline(self, image):
        pre_image = image
        # Perspective Transform
        image = self.unwarp(image)
        # HLS L-channel Threshold
        image_L = self.hls_lthresh(image)
        # Lab B-channel Threshold
        image_B = self.lab_bthresh(image)
        # Combine HLS and Lab B channel thresholds
        combined = np.zeros_like(image_B)
        combined[(image_L == 1) | (image_B == 1)] = 1

        f, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 9))
        ax1.imshow(pre_image)
        ax2.imshow(image_L)
        ax3.imshow(image_B)
        f.savefig('image.png')
        plt.close(f)

        return combined

    # Define method to fit polynomial to binary image with lines extracted, using sliding window
    def sliding_window_polyfit(self, img):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(img[img.shape[0]//2:,:], axis=0)
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]//2)
        quarter_point = np.int(midpoint//2)
        # Previously the left/right base was the max of the left/right half of the histogram
        # this changes it so that only a quarter of the histogram (directly to the left/right) is considered
        leftx_base = np.argmax(histogram[quarter_point:midpoint]) + quarter_point
        rightx_base = np.argmax(histogram[midpoint:(midpoint+quarter_point)]) + midpoint

        # Choose the number of sliding windows
        nwindows = 15
        # Set height of windows
        window_height = np.int(img.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 80
        # Set minimum number of pixels found to recenter window
        minpix = 40
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []
        # Rectangle data for visualization
        rectangle_data = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window+1)*window_height
            win_y_high = img.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            rectangle_data.append((win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high))
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        left_fit, right_fit = (None, None)
        # Fit a second order polynomial to each
        if len(leftx) != 0:
            left_fit = np.polyfit(lefty, leftx, 2)
        if len(rightx) != 0:
            right_fit = np.polyfit(righty, rightx, 2)
        
        visualization_data = (rectangle_data, histogram)
        
        return left_fit, right_fit, left_lane_inds, right_lane_inds, visualization_data

    # Define method to fit polynomial to binary image based upon a previous fit (chronologically speaking);
    # this assumes that the fit will not change significantly from one video frame to the next
    def polyfit_using_previous_fit(self, binary_warped, left_fit_prev, right_fit_prev):
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 80
        left_lane_inds = ((nonzerox > (left_fit_prev[0]*(nonzeroy**2) + left_fit_prev[1]*nonzeroy + left_fit_prev[2] - margin)) & 
                        (nonzerox < (left_fit_prev[0]*(nonzeroy**2) + left_fit_prev[1]*nonzeroy + left_fit_prev[2] + margin))) 
        right_lane_inds = ((nonzerox > (right_fit_prev[0]*(nonzeroy**2) + right_fit_prev[1]*nonzeroy + right_fit_prev[2] - margin)) & 
                        (nonzerox < (right_fit_prev[0]*(nonzeroy**2) + right_fit_prev[1]*nonzeroy + right_fit_prev[2] + margin)))  

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        left_fit_new, right_fit_new = (None, None)
        if len(leftx) != 0:
            # Fit a second order polynomial to each
            left_fit_new = np.polyfit(lefty, leftx, 2)
        if len(rightx) != 0:
            right_fit_new = np.polyfit(righty, rightx, 2)
        return left_fit_new, right_fit_new, left_lane_inds, right_lane_inds


# Define a class to receive the characteristics of each line detection
class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        #polynomial coefficients for the most recent fit
        self.current_fit = []  
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #number of detected pixels
        self.pixel_count = None

    def add_fit(self, fit, inds):
        # add a found fit to the line, up to n
        if fit is not None:
            if self.best_fit is not None:
                # if we have a best fit, see how this new fit compares
                self.diffs = abs(fit-self.best_fit)
            if (self.diffs[0] > 0.001 or \
               self.diffs[1] > 1.0 or \
               self.diffs[2] > 100.) and \
               len(self.current_fit) > 0:
                # bad fit! abort! abort! ... well, unless there are no fits in the current_fit queue, then we'll take it
                self.detected = False
            else:
                self.detected = True
                self.pixel_count = np.count_nonzero(inds)
                self.current_fit.append(fit)
                if len(self.current_fit) > 5:
                    # throw out old fits, keep newest n
                    self.current_fit = self.current_fit[len(self.current_fit)-5:]
                self.best_fit = np.average(self.current_fit, axis=0)
        # or remove one from the history, if not found
        else:
            self.detected = False
            if len(self.current_fit) > 0:
                # throw out oldest fit
                self.current_fit = self.current_fit[:len(self.current_fit)-1]
            if len(self.current_fit) > 0:
                # if there are still any fits in the queue, best_fit is their average
                self.best_fit = np.average(self.current_fit, axis=0)

class Driver(Node):
    turn = 0 # 0 for straight, <0 for left, >0 for right
    left = 1000
    right = 1000
    velocity = None
    init = None
    now = None
    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand,
                '/vehicle_cmd', 10)
        self.create_subscription(Odometry, '/lgsvl_odom',
                self.odom_callback, 10)
        self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
                self.camera_callback, 10)

        self.tmr = self.create_timer(0.01, self.controller_callback)

    def controller_callback(self):
        msg = RawControlCommand()
        msg.throttle = 20
        msg.front_steer = 0
        dis = 0
        if self.init and self.now:
            dis = abs(self.init.x - self.now.x)
        if self.velocity and ((self.velocity.x**2)+(self.velocity.y**2)+(self.velocity.z**2))**0.5 > 10.0/3.6:
            msg.throttle = 0
        if dis > 17 and (self.left < 20000 or self.right < 20000):
            if self.turn < 0 and self.left:
                msg.front_steer = int(-50*1000.0/float(self.left))
            elif self.turn > 0 and self.right:
                msg.front_steer = int(30*1000.0/float(self.right))
        self.pub.publish(msg)

    def odom_callback(self, data):

        position = data.pose.pose.position
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #TODO
        velocity = data.twist.twist.linear
        self.velocity = velocity
        if not self.init:
            self.init = position
        else:
            self.now = position

    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        #print (image_data)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        #print('image format: ', image_format,', size: ',image.shape)
        #TODO
        def pipeline(original_image):
            image = np.copy(original_image)
            bird_view_image = helper.pre_pipeline(image)

            if left_lane.detected and right_lane.detected:
                left_fit, right_fit, left_lane_indices, right_lane_indices = helper.polyfit_using_previous_fit(bird_view_image, left_lane.best_fit, right_lane.best_fit)
            else:
                left_fit, right_fit, left_lane_indices, right_lane_indices, _ = helper.sliding_window_polyfit(bird_view_image)

            if left_fit is not None and right_fit is not None:
                height = image.shape[0]
                left_fit_x_intercept = left_fit[0] * height ** 2 + left_fit[1] * height + left_fit[2]
                right_fit_x_intercept = right_fit[0] * height ** 2 + right_fit[1] * height + right_fit[2]
                x_intercept_diff = abs(right_fit_x_intercept - left_fit_x_intercept)
                '''
                if abs(350 - x_intercept_diff) > 100:
                    left_fit = None
                    right_fit = None
                '''

            left_lane.add_fit(left_fit, left_lane_indices)
            right_lane.add_fit(right_fit, right_lane_indices)

            if left_lane.best_fit is not None :
                left_curvature, right_curvature, distance_from_center = helper.calc_curvature_and_center_dist(bird_view_image, left_lane.best_fit, right_lane.best_fit, left_lane_indices, right_lane_indices)
                print('left = %f, right = %f, distance = %f' %(left_curvature, right_curvature, distance_from_center))
                return left_curvature, right_curvature, distance_from_center
            else:
                return 1000, 1000, 0

        # Init Helper
        helper = Helper(image)
        left_lane = Line()
        right_lane = Line()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.left, self.right, self.turn = pipeline(image)
        if self.left:
            print('turn = %f, steer = %d' %(self.turn, int(-50*1000.0/float(self.left))))


rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


