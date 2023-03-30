#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class JetTurtleTwistCTRL:
    def __init__(self):
        #init
        rospy.loginfo("Starting jet_turtle twist_ctrl.py.py")
        
        self.left_track_pub = rospy.Publisher("left_track", Float32, queue_size=10)
        self.right_track_pub = rospy.Publisher("right_track", Float32, queue_size=10)

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

    def twist_callback(self, msg): 
        # rospy.loginfo("received twist")

        width = 1.0 #20cm
        wheel_radius = 0.60 #tuning parameter

        #apply kinematics of a diff drive robot
        left_speed = (1/wheel_radius) * (msg.linear.x - (width * msg.angular.z) / 2)
        right_speed = (1/wheel_radius) * (msg.linear.x + (width * msg.angular.z) / 2)

        left_cmd = Float32()
        right_cmd = Float32()

        left_cmd.data = left_speed
        right_cmd.data = right_speed

        self.left_track_pub.publish(left_cmd)
        self.right_track_pub.publish(right_cmd)
        


# initialize node when script is called
if __name__ == '__main__':
    rospy.loginfo('starting jet_turtle twist_ctrl.py.py')
    rospy.init_node('jet_turtle twist_ctrl.py', log_level=rospy.INFO)

    try:
        node = JetTurtleTwistCTRL()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logfatal('caught exception')
    
    rospy.loginfo('exiting')
