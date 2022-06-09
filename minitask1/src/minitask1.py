#!/usr/bin/ env python
from typeshed import Self
import rospy
from geometry_msgs.msg import Twist
import time

class MoveTb2():

    def __init__(self):
        self.tb2_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size =1)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(10) #hz


    def publish_once_in_cmd_vel(self, cmd):
        while not self.ctrl_c:
            connections = self.tb2_vel_publisher.get_num_connections()
            if connections > 0:
                self.tb2_vel_publisher.publish(cmd)
                rospy.loginfo("cmd published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
    #works better than the rospy.is shut_down()
        self.stop_tb2()
        self.ctrl_c = True

    def stop_tb2(self):
        rospy.loginfo("Shutdown time stop robot")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.puplish_once_in_cmd_vel(cmd)

    def move_x_time(Self,moving_time, linear_speed = 0.2, angular_speed = 0.2):
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        rospy.loginfo("Moving Fowards")
        self.publish_once_in_cmd_vel(cmd)
        time.sleep(moving_time)
        Self.stop_tb2()
        rospy.loginfo("######## finished moving foward")

    def move_square(self):
        i = 0
        while not self.ctrl_c and i < 4:
            ##moving forward
            self.move_x_time(moving_time=2.0, linear_speed= 0.2, angular_speed=0.0)
            #Stop
            self.move_x_time(moving_time=4.0, linear_speed= 0.0, angular_speed = 0.0)
            #turn
            self.move_x_time(moving_time=3.5, linear_speed= 0.0, angular_speed = 0.2)
            #Stop
            self.move_x_time(moving_time=0.1, linear_speed= 0.0, angular_speed = 0.2)

            i += 1
        rospy.loginfo("##### Finished moving in square")


if __name__ == '__main__':
    rospy.init_node("move_tb2_test", anonymous = True)
    moveTb2_object = MoveTb2()
    try:
        moveTb2_object.move_square()
    except rospy.ROSInterruptException:
        pass
