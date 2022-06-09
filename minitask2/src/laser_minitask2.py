#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    result = msg.ranges[180]
    print(result)

rospy.init_node('laser_readings')
laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)
rospy.spin()


