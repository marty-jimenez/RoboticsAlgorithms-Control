#!/usr/bin/env python3

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt
import math

 
# angle and distant difference constraints
# you can adjust the values for better performance
angle_eps = 0.07
dis_eps = 0.09

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle',
    2: 'go back to closest point',
}
# define initial scenario
state = 0


def main():
    global pub
    global state

    # initialize ROS node
    rospy.init_node("bug_1")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(80)

    # Set the goal point 
    goal = Point()
    goal.x = -1.0
    goal.y = 1.0

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 

    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()
    i = 0
    count = 0
    hit_count = 0
    
    while not rospy.is_shutdown():
        # Decide what to do for the robot in each of these states:

        # the x,y distance to the goal from current position
        inc_x = goal.x - odom.x
        inc_y = goal.y - odom.y

        # the angle of the goal point wrt global frame
        angle_to_goal = atan2(inc_y, inc_x)

        # the distance to the goal from current location
        dist_diff = sqrt(inc_x**2 + inc_y**2)

        # find the heading angle difference
        angle_diff = angle_to_goal - odom.theta

        if state == 0:
            # go to goal state. 
            '''
            Here robot should go towards a the goal unless it encounters an obstacle.
            When it encounters the wall it should update the hit_point, and change the state to 
            "circumnavigate obstacle".

            It's an updated version of the "go_to_point.py"
            '''
            
            # adjust angle
            if angle_diff > angle_eps:
              speed.linear.x = 0.0
              speed.angular.z = 0.1
            elif angle_diff < -angle_eps:
              speed.linear.x = 0.0
              speed.angular.z = -0.1
            # go to the point
            elif dist_diff > dis_eps:
              speed.linear.x = 0.15
              speed.angular.z = 0.0
            # arrived
            else: 
              speed.linear.x = 0.0
              speed.angular.z = 0.0
              print("\n\nArrived!\n\n")
              print("\nx: %.3f, \t y: %.3f, \ttheta: %.3f" %(odom.x, odom.y, odom.theta))
              break
            
            # if we're near a wall
            if scan.region['front'] <= 0.27:
              speed.linear.x = 0.0  # stop
              speed.angular.z = 0.0
              hit_point.x = odom.x # update our hit point
              hit_point.y = odom.y
              
              # no solution terminating
              hit_count = hit_count + 1
              if hit_count > 1:
              	print("\n\nHit object more than once, terminating...")
              	break
              
              state = 1     # go-to circumnavigate obstactle 
            print("current state: ", state_dict[state])

        elif state == 1:
            # circumnavigate obstacle. 
            '''
              Here robot should turn right/left based on your choice. And, circumnavigate the obstacle using wall following
              algorithm from previous project. 
                
              While in this state, record closest point to goal where you can head towards goal.
                
              This state terminates when you reach the same point when you hit the obstacle.

              Finally, change the state.

              It's an updated version of the "follow_wall.py"
            '''
            # if we're not close to a wall
            if scan.region['front'] > 0.27:
              # if we're too far from the wall
              if scan.region['fleft'] > 0.27:
                speed.linear.x = 0.0
                speed.angular.z = 0.15   # adjust
                if dist_diff < closest_dist: # if this is the closest distance 
                  closest_dist = dist_diff   # update everything accordingly
                  closest_point.x = odom.x  
                  closest_point.y = odom.y
              elif scan.region['fleft'] <= 0.27 and scan.region['fleft'] > 0.18: # good range to just go straight
                speed.linear.x = 0.15 # go straight
                speed.angular.z = 0.0
                if dist_diff < closest_dist: 
                  closest_dist = dist_diff
                  closest_point.x = odom.x
                  closest_point.y = odom.y
              else:
                speed.linear.x = 0.0
                speed.angular.z = -0.15 # adjust
                if dist_diff < closest_dist: 
                  closest_dist = dist_diff
                  closest_point.x = odom.x
                  closest_point.y = odom.y
            else:
              speed.linear.x = 0.0
              speed.angular.z = -0.15 # turn right
              if dist_diff < closest_dist: 
                closest_dist = dist_diff
                closest_point.x = odom.x
                closest_point.y = odom.y

            # if we reach hitpoint distance
            tempx =     hit_point.x - odom.x
            tempy =     hit_point.y - odom.y
            tempDist =  sqrt(pow(tempx, 2) + pow(tempy, 2))
            if tempDist <= dis_eps and count > 800: # if we're close and this isn't our first time here (initial hit_point problem fix)
              speed.linear.x = 0.0
              speed.angular.z = 0.0
              state = 2             	# go back to closest point
            print("current state: ", state_dict[state])
            count = count + 1 	# counter used for initial hit point error


        elif state == 2:
            # go back to closest point
            '''
            Here robot should go back to closest point encountered in state 1. 
            Once you reach that point, change the state to "go to goal".

            It's an updated version of the "follow_wall.py"
            ''' 

            #unless the robot's front part gets too close to the wall
            if scan.region['front'] > 0.3:
                #if it goes too far from the wall or the left side 
                #is empty, turn the robot to the left
                if scan.region['fleft'] > 0.25:
                    speed.angular.z = 0.15
                    speed.linear.x = 0
                #when the left side of the robot is relatively close to the wall,
                #let the robot move straight
                elif scan.region['fleft'] <= 0.25 and scan.region['fleft'] > 0.18:
                    speed.angular.z = 0
                    speed.linear.x = 0.15
                #when the robot gets too close to the wall, turn the robot to the right
                else:
                    speed.angular.z = -0.15
                    speed.linear.x = 0
            #if the front part of the robot meets the wall, turn to the right
            else:
                speed.angular.z = -0.15
                speed.linear.x = 0
            
            tempx2 = closest_point.x - odom.x
            tempy2 = closest_point.y - odom.y
            tempDist2 = sqrt(pow(tempx2, 2) + pow(tempy2, 2))
            if tempDist2 <= dis_eps: # if we're close to closest point
              speed.linear.x = 0.0
              speed.angular.z = 0.0
              state = 0             # go to goal
            print("current state: ", state_dict[state])

        print(scan.region)
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
