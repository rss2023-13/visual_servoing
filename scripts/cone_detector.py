#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        #self.desired_turns = ["left", "right", "right", "left"]
        #self.last_state = "straight"
        #self.current_turn = 0

    def slice_img(self, img, lower, upper): # replace 
        img[:int(lower*img.shape[0])] = (0,0,0)
        img[int(upper*img.shape[0]):] = (0,0,0)

        return img
    
    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        image = self.slice_img(image, .7, .9) # for line following
        
        top_left, bottom_right = cd_color_segmentation(image)
        total_width = image.shape[1] # width of full image

        box_width = bottom_right[0] - top_left[0]
        #print(total_width, box_width)
        cone_loc = ConeLocationPixel()
        cone_loc.v = top_left[1]
 #       cone_loc.u = top_left[0] + box_width/2
        
        #print("current:", self.last_state, ", on turn #", self.current_turn)

        if box_width / float(total_width) > 1./3:# detecting turn
            #if bottom_right[0] > total_width*2/3 and top_left[0] > total_width/4: # tape goes to the right
            #print(total_width-bottom_right[0], top_left[0])

            if total_width - bottom_right[0] < top_left[0] and bottom_right[0] > total_width * 2/3. and top_left[0] > 1/3.:# and self.current_turn in (1, 2): #right turn
                cone_loc.u = top_left[0] + box_width * 4/5.
                print("detecting right turn")
                #if "right" != self.last_state:
                    #self.last_state = "right"

            elif total_width - bottom_right[0] > top_left[0] and top_left[0] < total_width * 1/3. and bottom_right[0] > 2/3.:# and self.current_turn in (0,3): # tape goes to the left
                cone_loc.u = top_left[0] + box_width * 1/5.
                print("detecting left turn")
                #if "left" != self.last_state :
                    #self.last_state = "left"
                    

            else:
                cone_loc.u = top_left[0] + box_width/2
                print("go straight2")
                #if "straight" != self.last_state:
                    #self.last_state = "straight"
                    #self.current_turn += 1 # increment next turn to look for
        else:
            cone_loc.u = top_left[0] + box_width/2
            #if "straight" != self.last_state:
                #self.last_state = "straight"
                #self.current_turn += 1
            print("go straight")

        self.cone_pub.publish(cone_loc)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
