#! /usr/bin/env python
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rospy
import random

def walkrandomly(move):
    t0 = rospy.Time.now().to_sec()
    move.linear.x = 0.5*random.random()
    move.angular.z = 0.5*random.random()
    rotate_time = 1*random.random()




    


def callback(data):
    front = min(data.ranges[0:5],data.ranges[355:359])
    right = min(data.ranges[30:40])
    left = min(data.ranges[330:340])
    print('****************************************************')
    print('Closes obstacle  at 0 degress: {}'.format(front))
    print("Closes obstacle at 15 degress: {}".format(right))
    print("Closes obstacle at 345 degress: {}".format(left))
    print('*****************************************************')

    # data.ranges[0])) = straigth forward
    # data.ranges[15])) = data from the right hand
    # data.ranges[345])) = data from the left hand

    ob_1 = 0.5  # Laser scan range
    t0 = rospy.Time.now().to_sec()


    if front > ob_1 and right > ob_1 and left > ob_1:
        
        move.linear.x = 0.5  # go forward (linear velocity)
        move.angular.z = 0.0  # do not rotate (angular velocity)
        pub.publish(move)
        t1 = rospy.Time.now().to_sec()
        distance_moved = (t1-t0)* move.linear.x

        while distance_moved >= 3.0:
            if data.ranges[15] > data.ranges[345]: # make the choice of direction to rotate
                move.linear.x = 0.0  # stop (linear velocity)
                # rotate clock-wise (angular velocity)
                move.angular.z = -abs(0.5*random.random())# random angular speed
            else:
                move.linear.x = 0.0  # stop (linear velocity)
                # rotate counter-clockwise (angular velocity)
                move.angular.z = 0.5*random.random()

    else:
        move.linear.x = 0.0  # stop
        move.angular.z = 0.5*random.random()  # rotate counter-clockwise
        if data.ranges[0] > ob_1 and data.ranges[15] > ob_1 and data.ranges[345] > ob_1:
            move.linear.x = 0.5  # go forward (linear velocity)
            move.angular.z = 0.0  # do not rotate (angular velocity)
    pub.publish(move)  # publish the move


move = Twist()  # Creates a Twist messaage
rospy.init_node('avoid_objects')  # initialle the node
# publishing twist message on cmd_vel topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
# listens to LaserScan messages from  from the scan topic && call the callback function any time it reads from the /scan topic
sub = rospy.Subscriber('/scan', LaserScan, callback)

rospy.spin()  # loops untill someones quites the application
