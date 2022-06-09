
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from math import radians, degrees,pi
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import numpy as np
import cv2,cv_bridge
import time

class searchObj:
    global front
    global left
    global right
    
    global mtgflag
    global h,w,d
    #global mtoflag
    global mask
    global front_dis
    global left_dis
    global right_dis
    global back_dis  
    mask = 0
    global rPointtest # if ther have a red obj 
    rPointtest = False    
    global bPointtest 
    bPointtest = False 
    global gPointtest 
    gPointtest = False 
    global centerPointlist
    centerPointlist = []
    global rn,bn,gn #count how many different color objects has been found when moving in the map
    global rerrspeed
    global gerrspeed
    global berrspeed
    rerrspeed = [0,0]
    gerrspeed = [0,0]
    berrspeed = [0,0]

    rn = 0
    bn = 0
    gn = 0

    #global mask
    def __init__(self) :
      self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
      self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
      self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.findObstcal) #  listens to LaserScan messages from 

      self.minRange = 1
      self.front = 0
      self.cXr = 0
      self.cYr = 0
      self.cXb = 0
      self.cYb = 0
      self.cXg = 0
      self.cYg = 0

      #self.rotateAng = pi
      #Stop at this distance
      self.minDistance = 0.5           # Minimum Distance between robot and obstacles      
      self.twist = Twist()             # Linear component velocities
      self.stop = False
      
      #start moveMain Function
      n = 0 #(initilise point)
      reachflag = True #  im already in the first point
      #new pointlist
      pointList= [[-0.994318246841,-1.94318139553],[0.0625000745058,-0.295454233885],
                 [0.642044842243,-3.64772701263],[3.03977274895,-0.931818187237],
                 [5.50759553909,-3.55273365974],[5.72745609283,3.63449859619],[5.37317848206,-1.90327870846]]

      pointlistzw = [[-0.707106796641,0.707106765732],[0.0625000745058,-0.295454233885],
                    [0.642044842243,-3.64772701263],[3.03977274895,-0.931818187237],
                    [5.50759553909,-3.55273365974],[0.693678612171,0.720284654159]]
      #pointList = [[-1.016,-2.826],[0.103,-0.254],[-1.103,-0.254],[3.657,-4.174]]
      #pointList = [[-1.016,-2.826],[0.103,-0.254],[3.995,1.779],[3.657,-4.174],[6.022,-4.249],[6.022,3.829]]
      self.moveMain2(pointList,pointlistzw,n)
  
    # processing images from camera
    def moveToGoal(self,xGoal,yGoal,zGoal,wGoal,n):
        global mtgflag
        mtgflag = False
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()
        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = zGoal
        goal.target_pose.pose.orientation.w = wGoal

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)
        ac.wait_for_result(rospy.Duration(80))
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
                rospy.loginfo("You have reached the {} point".format(n))
                mtgflag = True
                #return True

        else:           
                rospy.loginfo("The robot failed to reach the destination")
                mtgflag = False

        print('check 109 line mtgflag is {}'.format(mtgflag))

              #return False
    # from the callback, return objlist: contain mask and T/F
    def callback(self, msg):# from the callback, return objlist: contain mask and T/F 
        
        global rPointtest # if there is a red obj in image
        global bPointtest # if there is a blue obj in image
        global gPointtest
        global centerPointlist
        global mask
        global rerrspeed
        global gerrspeed
        global berrspeed
        linearx = 0.2

        self.bridge = cv_bridge.CvBridge()
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #cv2.imshow("original", image)
        global h,w,d
        h,w,d = image.shape
        image_resized = cv2.resize(image, (w/4,h/4))
        cv2.imshow("small_original", image_resized)
        cv2.waitKey(30)
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

        # setting bondaries for Green color 
        greenLower = np.array([50,230,30])
        greenUpper =np.array([70,255,120])
        # getting only green values from the camera
        gmask = cv2.inRange(hsv,greenLower, greenUpper)
        gM = cv2.moments(gmask)
        #cv2.imshow("gM",gmask)
        cv2.waitKey(25)

        # setting bondaries for Red color 
        redLower1 = np.array([0,245,30])
        redUpper1 = np.array([6,255,130])
        redLower2 = np.array([174,245,30])
        redUpper2 = np.array([180,255,130])
        # getting only Red values from the camera
        rmask1 = cv2.inRange(hsv,redLower1, redUpper1)
        rmask2 = cv2.inRange(hsv,redLower2, redUpper2)
        #rmask =  rmask2
        rmask = rmask1 | rmask2
        rM = cv2.moments(rmask)
        #cv2.imshow("rM",rmask)
        cv2.waitKey(25)

        # setting bondaries for blue color 
        blueLower = np.array([105,130,30])
        blueUpper = np.array([120,170,50])
        # getting only Red values from the camera
        bmask = cv2.inRange(hsv,blueLower, blueUpper)
        bM = cv2.moments(bmask)
        #cv2.imshow("bM",bmask)
        cv2.waitKey(25)
        #adding all colors as one  
        mask = gmask + rmask + bmask
        self.goalReached = False
        cv2.imshow("rgb",mask)


        if rM['m00'] > 100:
            rPointtest = True
            self.cXr = int(rM["m10"] /rM["m00"])
            self.cYr = int(rM["m01"] / rM["m00"]) 
            rerrspeed[0]=linearx
            rerrspeed[1]=-float(self.cXr - w/2)/1000/2
            #rerrspeed=[linearx,-float(self.cXr - w/2)/1000/2]    
        else: 
            rPointtest = False

        if bM['m00'] > 100:
            bPointtest = True
            self.cXb = int(bM["m10"] / bM["m00"])
            self.cYb = int(bM["m01"] / bM["m00"])
            berrspeed[0]= linearx
            berrspeed[1]= -float(self.cXb - w/2)/1000/2
            #rerrspeed=[linearx,- float(self.cXb - w/2)/1000/2]
        else: 
            bPointtest = False

        if gM['m00'] > 100:
            gPointtest = True
            self.cXg = int(gM["m10"] / gM["m00"])
            self.cYg = int(gM["m01"] / gM["m00"])
            gerrspeed[0]=linearx
            gerrspeed[1]=-float(self.cXg - w/2)/1000/2
            #rerrspeed=[linearx,- float(self.cXg - w/2)/1000/2]
        else:
            gPointtest = False
        cv2.waitKey(25)
        
        #return objlist

    #choose obj to move
    # move to the target with obstacle avoidance
    def movechoiceObj(self):
        print('lineflag 199 running movechoiceObj')
        global rPointtest # if there is a red obj in image
        global bPointtest # if there is a blue obj in image
        global gPointtest
        global front_dis
        global left_dis
        global right_dis
        global back_dis
        global w
        global rn,bn,gn #count how many different color objects has been found when moving in the map
        # global mtpflag
        global mask

        if np.sum([rPointtest,bPointtest,gPointtest]) == 1: # if there have one obj in image     
            if front_dis > self.minDistance : # <=
                if rPointtest:
                    self.moveWithAviod(0)
                    self.cmd_vel_pub.publish(self.twist)
                    while front_dis == 0.6:
                        rn += 1
                        print("lineflag 224 there is the {} RED object".format(rn))
                
                elif bPointtest:
                    self.moveWithAviod(1)
                    while front_dis == 0.6:
                        bn += 1
                        print("lineflag 233 there is the {} BLUE object".format(bn))
                         
                
                elif gPointtest:
                    self.moveWithAviod(2)
                    while front_dis == 0.6:
                        gn +=1
                        print("lineflag 243 there is the {} GREEN object".format(gn))

            else:
                if rPointtest:
                    rn += 1
                    print("lineflag 270 there is the {} RED object".format(rn))

                if gPointtest:
                    gn +=1
                    print("lineflag 274 there is the {} GREEN object".format(gn))
            
                if bPointtest:
                    bn += 1
                    print("lineflag 278 there is the {} BLUE object".format(bn))


        elif np.sum([rPointtest,bPointtest,gPointtest]) > 1:# more than to obj in the screen , at least 2 objs in image
            if front_dis > self.minDistance :
                if self.cYr > self.cYb and self.cYr > self.cYb: # we choose the nearst obj, and that obj has the smallest cY value
                    self.moveWithAviod(0) # move to red obj 
                    while front_dis == 0.6:
                        rn += 1
                        print("lineflag 284 there is the {} RED object".format(rn))

                elif self.cYb > self.cYr and self.cYb > self.cYg: # we choose the nearst obj, and that obj has the smallest cY value
                    self.moveWithAviod(1) # move to blue obj 
                    while front_dis == 0.6:
                        bn += 1
                        print("lineflag 293 there is the {} BLUE object".format(bn))

                elif self.cYg > self.cYr and self.cYg > self.cYb: # we choose the nearst obj, and that obj has the smallest cY value
                    self.moveWithAviod(2) # move to green obj 
                    while front_dis == 0.6:
                        gn +=1
                        print("lineflag 302 there is the {} GREEN object".format(gn))
            else:
                if self.cYr > self.cYb and self.cYr > self.cYb:
                    rn += 1
                    print("lineflag 306 there is the {} RED object".format(rn))

                elif self.cYb > self.cYr and self.cYb > self.cYg:
                    bn += 1
                    print("lineflag 310 there is the {} BLUE object".format(bn))
                
                elif self.cYg > self.cYr and self.cYg > self.cYb:
                    gn +=1
                    print("lineflag 314 there is the {} GREEN object".format(gn))

    # rotate the robot 180 degree to search obj
    def rotate180(self,list,n):
        global rPointtest # if there is a red obj in image
        global bPointtest # if there is a blue obj in image
        global gPointtest # if there is a green obj in image
        rate = 50 
        # rate
        r = rospy.Rate(rate) 
        self.rotateAng = pi #rotate angular wanted
        self.twist.linear.x = 0.0  #linear speed
        self.twist.angular.z = pi/6 #angular speed
        self.rotateTime = pi*6
        startTime = time.time()
        #self.rotateTime = self.rotateAng/self.twist.angular.z #rotate time
        
        while (time.time()-startTime) <= self.rotateTime:

            self.cmd_vel_pub.publish(self.twist)

            if any([rPointtest,bPointtest,gPointtest]):
                print("lineflag 336: there is a obj")
                self.twist.linear.x = 0.0  #linear speed
                self.twist.angular.z = 0.0 #angular speed
                self.cmd_vel_pub.publish(self.twist)
                self.movechoiceObj()
                break
            else:
                self.cmd_vel_pub.publish(self.twist) #rotate move

        print("lineflag 345 , no obj and i have rotated 180")
    #move function, to control move and switch between global and local planer
    def moveMain2(self,list,listzw,n):
        global mtgflag
       
        while n <= len(list):
            x,y = list[n] 
            z,w = listzw[n]
            self.moveToGoal(x,y,z,w,n)
            n += 1
            print("lineflag 355 mtgflage is {}".format(mtgflag))
            if mtgflag: #if arrive the n waypoint
                self.rotate180(list,n) # start to rotate and search nearby obj
                print('lineflag 358 check if this line work')
                #mtgflag = False
                self.moveMain2(list,listzw,n)
            else:
                rospy.loginfo("lineflag 362 we failed to this point")
                self.moveMain2(list,listzw,n)
        else:
            print("lineflag 365 you have finished  this trip")

#--------------obstacle Avodiance---------------#
    #this is local move planner when find obstacle
    def moveWithAviod(self,pointflag):
        #print("lineflag 368linear speed is {}, anglar speed is {}".format(x,z))
        #linear_speed = x
        #angular_speed = -z
        global rerrspeed   #rerrspeed[0]  rerrspeed[1]
        global gerrspeed
        global berrspeed
        angularz = 0.2
        move = Twist()
        if pointflag == 0:# red 
            print("line 362 red test")  
            while front_dis > self.minDistance:      
                #print("lineflag 364 linear speed is {}, anglar speed is {}".format(rerrspeed[0],rerrspeed[1]))
                if left == True and front  == True and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    
                #2nd situation
                elif left == True and front  == True and right == True:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #3rd situation
                elif left == True and front  == False and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #4th situation
                elif left == True and front  == False and right == True:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #5 situation
                elif left  == False and front  == True and right == True:
                    move.linear.x = 0
                    move.angular.z = angularz
                    self.cmd_vel_pub.publish(move)
                    #6 situation
                elif left  == False and front == True and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #7 situation
                elif left  == False and front  == False and right == True:
                    move.linear.x = 0
                    move.angular.z = angularz
                    self.cmd_vel_pub.publish(move)
                    #8 situation just go ahend
                elif left  == False and front  == False and right == False:
                    move.linear.x = 0.2
                    move.angular.z = rerrspeed[1]
                    self.cmd_vel_pub.publish(move)
        
        elif pointflag == 1:# blue      
            print("lineflag 405 blue test")  
            while front_dis > self.minDistance: 
                #print("lineflag 390 linear speed is {}, anglar speed is {}".format(berrspeed[0],berrspeed[1]))
                if left == True and front  == True and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    
                #2nd situation
                elif left == True and front  == True and right == True:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #3rd situation
                elif left == True and front  == False and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #4th situation
                elif left == True and front  == False and right == True:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #5 situation
                elif left  == False and front  == True and right == True:
                    move.linear.x = 0
                    move.angular.z = angularz
                    self.cmd_vel_pub.publish(move)
                    #6 situation
                elif left  == False and front == True and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #7 situation
                elif left  == False and front  == False and right == True:
                    move.linear.x = 0
                    move.angular.z = angularz
                    self.cmd_vel_pub.publish(move)
                    #8 situation just go ahend
                elif left  == False and front  == False and right == False:
                    move.linear.x = 0.2
                    move.angular.z = berrspeed[1]
                    self.cmd_vel_pub.publish(move)

        if pointflag == 2:# green   
            print("lineflag 448 green test")
            while front_dis > self.minDistance:     
                #print("lineflag 416 linear speed is {}, anglar speed is {}".format(gerrspeed[0],gerrspeed[1]))
                if left == True and front  == True and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    
                #2nd situation
                elif left == True and front  == True and right == True:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #3rd situation
                elif left == True and front  == False and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #4th situation
                elif left == True and front  == False and right == True:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #5 situation
                elif left  == False and front  == True and right == True:
                    move.linear.x = 0
                    move.angular.z = angularz
                    self.cmd_vel_pub.publish(move)
                    #6 situation
                elif left  == False and front == True and right == False:
                    move.linear.x = 0
                    move.angular.z = -angularz
                    self.cmd_vel_pub.publish(move)
                    #7 situation
                elif left  == False and front  == False and right == True:
                    move.linear.x = 0
                    move.angular.z = angularz
                    self.cmd_vel_pub.publish(move)
                    #8 situation just go ahend
                elif left  == False and front  == False and right == False:
                    move.linear.x = 0.2
                    move.angular.z = rerrspeed[1]
                    self.cmd_vel_pub.publish(move)

    #this is lasercall callback function, receive laser data from there
    def findObstcal(self,data):
        global front
        global left
        global right
        global front_dis
        global left_dis
        global right_dis
        global back_dis
        left_dis = data.ranges[30]
        front_dis = data.ranges[0]
        back_dis = data.ranges[180]
        right_dis = data.ranges[330]

        limit_dist = 0.5
        ld = limit_dist
        ob_list = [False,False,False]
        left = (data.ranges[30] < ld) # front true means this direction has obstacle
        front = (data.ranges[0] < ld)
        right  = (data.ranges[330] < ld)
    
    #we will use many times to setup speed and publish speed messgae, so i create a function
    def moveControl(self,x,z):
        #init speed  1.0 with 0.0 angualr speed
        #lx = 0.2
        #az = 0.5
        move = Twist()
        move.linear.x = x
        move.angular.z = z
        self.cmd_vel_pub.publish(move)

    #obstacle avoiodance

              
rospy.init_node('Search')    # initialize the node name 
search = searchObj()         # creat a new object from our SearchObj class
rospy.Rate(10)               # Loop 10 times per second (10hz)
rospy.spin()  

