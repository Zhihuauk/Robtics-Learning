#!/usr/bin/env python

from rosgraph import roslogging
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random


class ObstacleAvoidance(object):

    def __init__(self):
        self.moved_distance =  Float64
        self.moved_distance.data = 0.0
        self.rate = rospy.Rate(50)
        self.get_init_position()
        move = Twist() # Creates a Twist messaage



        self.moved_distance_pub = rospy.Publisher('/moved_distance', Float64, queue_size=50)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.dist_walk_randomly_callback)

        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50) #publishing twist message on cmd_vel topic 
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.obstacle_callback) #  listens to LaserScan messages from  from the scan topic andand call the callback function any time it reads from the /scan topic 


    #get initi position
    def get_init_position(self):
        odom = None
        while odom is None:
            try:
                odom = rospy.wait_for_message('/odom',Odometry,timeout=1)
            except:
                rospy.loginfo('now odometery message now')
        #odom = self.odom_sub # sub from odom message

        self.current_position = Point() #typr of message contain float x,y,z
        self.current_position.x = odom.pose.pose.position.x
        self.current_position.y = odom.pose.pose.position.y
        self.current_position.z = odom.pose.pose.position.z
    # update the position
    def get_new_position(self,new_position):

        self.current_position.x = new_position.x
        self.current_position.y = new_position.y
        self.current_position.z = new_position.z

# calculate the distance between previous and new position
    def calcu_dist(self,previous_position,new_position):

        x1 = previous_position.x
        y1 = previous_position.y
        x2 = new_position.x
        y2 = new_position.y

        dist = math.sqrt((x2-x1)**2+(y2-y1)**2) 
        return dist

    def dist_walk_randomly_callback(self,msg):
        newPosition = msg.pose.pose.position
        self.moved_distance.data  += self.calcu_dist(self.current_position,newPosition)
        self.moved_distance_pub.publish(self.moved_distance) # publish the distance between previous and new position


    def obstacle_callback(self,data):
        front = data.ranges[0]
        right = data.ranges[15]
        left  = data.ranges[345]

        print ('****************************************************')
        print ('Closes obstacle  at 0 degress: {}'.format(front))
        print ("Closes obstacle at 15 degress: {}".format(right))
        print ("Closes obstacle at 345 degress: {}".format(left))
        print ("current Distance x: {}".format(self.moved_distance.data))
        print ('*****************************************************')

        limit_dist = 0.5
        move = Twist()
        #random = random.random()


      #  prince
        if front >limit_dist and right > limit_dist and left > limit_dist:
            move.linear.x = 0.3
            move.angular.z = 0.0
            self.vel_pub.publish(move)
            if (self.moved_distance.data > 3.0):
                if (left > right):
                    move.linear.x = 0.0
                    move.angular.z = -abs(0.5)
                    self.moved_distance.data = 0.0
                    self.vel_pub.publish(move)
                else:
                    move.linear.x = 0.0
                    move.angular.z = 0.5
                    self.moved_distance.data = 0.0
                    self.vel_pub.publish(move)
                
        
        
        else:
            move.linear.x = 0.0
            move.angular.z = 0.5 # rotate counter-clockwise
            if data.ranges[0] > limit_dist and data.ranges[15]>limit_dist and data.ranges[345]>limit_dist: 
                move.linear.x = 0.5 #go forward (linear velocity)
                move.angular.z = 0.0 # do not rotate (angular velocity)
        self.vel_pub.publish(move) # publish the move


if __name__ == '__main__':
    rospy.init_node('move_avoidAbstacle',anonymous=True)
    move= ObstacleAvoidance()
    rospy.spin()        
            
            

            


