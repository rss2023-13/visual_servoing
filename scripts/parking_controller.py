#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            Float32, queue_size=10)

        self.parking_distance = .25 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.steering_kp = 1.9
        self.steering_kd = 0.05
        self.steering_ki = 0

        self.velocity_kp = 1
        self.velocity_kd = 0
        self.velocity_ki = 0
        self.velocity_max = 0.5

	self.prev_error = 0
	self.prev_time = rospy.get_time()

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # Goal: Drive to a distance of self.parking_distance away from cone
        # Also, make sure to face the cone
        # Goal is to move the x_error and y_error to 0, where x_error = self.relative_x - self.parking_distance
        # and y_error = self.relative_y
        
        y_error = self.relative_y
        x_error = self.relative_x - self.parking_distance
        angle = np.arctan2(self.relative_y, self.relative_x)
        overall_error = np.linalg.norm([x_error, y_error])
        rospy.loginfo("angle = " + str(angle * 180 / np.pi))
        spin = (angle > np.pi / -4) and (angle < np.pi / 4)
        if  not spin:
            steering_angle = .7
            velocity = self.velocity_max
        else:
            # Set the steering angle
	    curr_time = rospy.get_time()
            steering_angle = self.steering_kp * angle + self.steering_kd * (angle - self.prev_error) / (curr_time - self.prev_time)
	    self.prev_time = curr_time
	    self.prev_error = angle


            # Set the velocity
#            if x_error >= 0:
#                velocity = self.velocity_kp * overall_error  #error_to_use
#            elif x_error > -.05:
#                velocity = self.velocity_kp * y_error
#            else:
#                velocity = self.velocity_kp * x_error


        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = self.velocity_max

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = Float32()
        #################################
	error_msg.data = self.relative_y
        # YOUR CODE HERE
        #error_msg.x_error = self.relative_x
        #error_msg.y_error = self.relative_y
        #error_msg.distance_error = np.sqrt(self.relative_x**2 + self.relative_y**2)

        #################################
        
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
