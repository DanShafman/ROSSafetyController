#!/usr/bin/python
#

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyControllerNode:
    def __init__(self): 
	# subscribe to incoming laser scan data
	rospy.Subscriber("/scan", LaserScan, self.callback)
	# subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)

        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)

    def ackermann_cmd_input_callback(self, msg):
        # republish the input as output (not exactly "safe")
        self.cmd_pub.publish(msg)

    def isClear(self, laser):	
	# If any object is found within a half meter in a 120deg angle, stop the vehicle
	for i in range(420, 660):
	    if laser.ranges[i] < 0.5:
		return False
	return True

    def callback(self, msg):
	# Completion handler for laser scan data
	if self.isClear(msg):
	    print("go")
	else:
	    print("stop")

if __name__ == "__main__":
    rospy.init_node("safety_controller")
    
    node = SafetyControllerNode()
    
    rospy.spin()
