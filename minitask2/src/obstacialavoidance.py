#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class circling():
    def __init__(self):
        global circle
        circle = Twist()
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.sub = rospy.Subscriber("/odm",Odometry, self.odometry)

    def callback(self, msg):
        print('Lidar sensor sensor data')
        print('front: {}'.format(msg.ranges[0]))
        print('left: {}'.format(msg.ranges[90]))
        print('back: {}'.format(msg.ranges[180]))
        print('right: {}'.format(msg.ranges[270]))
        


        self.distance = 0.5
        if msg.ranges[0]>self.distance and msg.ranges[90]>self.distance and msg.ranges[180]>self.distance and msg.ranges[270]>self.distance:

            circle.linear.x = 0.5
            circle.angular.z = 0.1
            rospy.loginfo("Circling")
        else:
            rospy.loginfo("obstacle detected")
            circle.linear.x = 0
            circle.angular.z = 0.5
            if msg.ranges[0]>self.distance and msg.ranges[15]>self.distance and msg.ranges[345]>self.distance and msg.ranges[45]>self.distance and msg.range[315]>self.distance:

                circle.linear.x = 0.5
                circle.angular.z = 0.1
        self.pub.publish(circle)# public the move object
        rospy.Rate(1)
    def odometry(self,msg):
        print msg.pose.pose  #??????
if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    circling()
    rospy.spin()