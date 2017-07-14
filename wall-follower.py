
#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollowingNode:
    def __init__(self):

	self.SIDE = "RIGHT"   
	self.RIGHT_ANGLE = 180
	self.LEFT_ANGLE = 940
	self.SPEED = 3

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher("ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

        # This is the distance from the wall you want to drive
        self.reference_cmd = 0.45

        # Desired speed you want the car to drive.
        self.speed_cmd = 1

	self.kp = 1

	self.u_steer = 1

    def laser_callback(self,msg):
        #Copy over to local variables
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges

        #Find a way to compute the error and error rate
        if self.SIDE == "RIGHT":
	    error = ranges[self.RIGHT_ANGLE] - self.reference_cmd
	else:
            error = ranges[self.LEFT_ANGLE] - self.reference_cmd
	
	# P Control
	u_input = self.kp * error

	output_msg = AckermannDriveStamped()
	output_msg.drive.speed = self.SPEED
	if self.SIDE == "RIGHT":
	    output_msg.drive.steering_angle = -u_input
      	    print(ranges[self.RIGHT_ANGLE], self.kp, error, u_input)
	else:
	    print(ranges[self.LEFT_ANGLE], self.kp, error, u_input)
