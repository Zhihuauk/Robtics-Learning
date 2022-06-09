#!/usr/bin/env python2
#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist

class hello():

    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Robot Started moving")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(50)
        self.rate.sleep()

        x = 0
        while x<14:
            self.move_robot(0.1,0.0,10.1)
            self.move_robot(0.0,0.1,16.0)
            x +=1
        self.move_robot(0.0,0.0,1)



    def move_robot(self, linear_x_value, angular_z_value, num_secs):
        turtle_vel = Twist()
        turtle_vel.linear.x = linear_x_value
        turtle_vel.angular.z = angular_z_value

        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec():
            self.vel_pub.publish(turtle_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        whatever = hello()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")