#!/usr/bin/env python
from genpy import message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rospy
#subcriber two topic
import message_filters
import math

def random_worlk(msg):
    dist_moved  = msg.data
    if dist_moved <= 3.0:
        move.linear.x = 0.5*random.random()
        move.angular.z = 0
    else:
        move.linear.x = 0
        move.angular.z = 0.5*random.random()
    pub.publish(move)

def dist_calcu(new_position,currently_position):



        





def callback(data):
    print ('****************************************************')
    print ('Closes obstacle  at 0 degress: {}'.format(data.ranges[0]))
    print ("Closes obstacle at 15 degress: {}".format(data.ranges[15]))
    print ("Closes obstacle at 345 degress: {}".format(data.ranges[345]) )
    print ('*****************************************************')
    #

    # data.ranges[0])) = straigth forward
    # data.ranges[15])) = data from the right hand 
    # data.ranges[345])) = data from the left hand

    ob_1 = 0.7 # Laser scan range  

    if data.ranges[0] > ob_1 and data.ranges[15] > ob_1 and data.ranges[345] > ob_1: 
        move.linear.x = 0.5 #go forward (linear velocity)
        move.angular.z = 0.0 # do not rotate (angular velocity)
        while msg.data >= 3:
            if data.ranges[15] > data.ranges[345]:
                move.linear.x = 0.0 # stop (linear velocity)
                move.angular.z = -abs(0.5) # rotate clock-wise (angular velocity)
            else:
                move.linear.x = 0.0 # stop (linear velocity)
                move.angular.z = 0.5 # rotate counter-clockwise (angular velocity)

   
    else:
        move.linear.x = 0.0 #stop
        move.angular.z =0.5 # rotate counter-clockwise
        if data.ranges[0] > ob_1 and data.ranges[15]>ob_1 and data.ranges[345]>ob_1: 
             move.linear.x = 0.5 #go forward (linear velocity)
             move.angular.z = 0.0 # do not rotate (angular velocity)
    pub.publish(move) # publish the move 



move = Twist() # Creates a Twist messaage
rospy.init_node('avoid_objects') #initialle the node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50) #publishing twist message on cmd_vel topic 
sub = rospy.Subscriber('/scan', LaserScan, callback) #  listens to LaserScan messages from  from the scan topic && call the callback function any time it reads from the /scan topic 

##subscriber two topic simutaneously
#sub1 = message_filters.Subscriber('/scan',LaserScan)
sub2 = message_filters.Subscriber('/move_distance',Float64)

#sub1_2 = message_filters.TimeSynchronizer([LaserScan,Float64],queue_size =10)
#sub1_2.registerCallback(callback)
rospy.spin() # loops untill someones quites the application