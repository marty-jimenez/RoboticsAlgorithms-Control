#!/usr/bin/env python3

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, pi

 
# angle and distance constraints
# you can adjust them if necessary
angle_eps = 0.2
dis_eps = 0.01

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
    0: 'go to point',
    1: 'wall detected',
    2: 'follow the wall',
}
# define initial scenario
state = 0


def main():
    global pub

    # initialize ROS node
    rospy.init_node("follow_wall")
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
    rate = rospy.Rate(100)

    # Set the goal point 
    goal = Point()
    goal.x = 0.0
    goal.y = 5.0
    state = 0

    # idk sort of struggles to wrap go around corner
    while not rospy.is_shutdown():
        # initialize speed as Twist topic
        speed  = Twist()
        # TODO:
        # Decide what to do for the robot in each of these states:

        if state == 0:
            # go to point state. 
            '''
            Hint: 
                Here robot should go towards a point located behind the wall. When it encounters the wall
                it should change the state to "wall detected".

                should be bunch of if-else statements

            '''
            if odom.theta < pi/2:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            else:
                speed.linear.x = 0.2
                speed.angular.z = 0.0

            if scan.region['front'] <= 0.3:
                speed.linear.x = 0.0
                speed.angular.z = 0.0 
                state = 1 # wall detected state  

            print("current state: ", state_dict[state])
 
        elif state == 1:
            # wall detected state. 
            '''
            Hint: 
                Here robot should turn right/left based on your choice. And, continue following the wall.
                Finally, do not forget to change the state!

                should be bunch of if-else statements

            '''
            if scan.region['front'] <= 0.45:
              speed.angular.z = -0.2
              speed.linear.x = 0.0
            else:
              speed.angular.z = 0.0
              speed.linear.x = 0.0
              state = 2 # change to wall-following state
            print("current state: ", state_dict[state])



        elif state==2:
            # wall following state. 
            '''
            Hint: 
                Here robot should go around the obstacle infinitely. Depending on the laserscan readings
                decide what to do for the robot.

                should be bunch of if-else statements
            '''
            if scan.region['front'] <= 0.45 and scan.region['fright'] <= 0.30:
              speed.linear.x = 0.0    # don't collide to wall when in pit-area
              speed.angular.z = -0.1
            elif scan.region['left'] < 0.30 and scan.region['left'] > 0.15 and scan.region['fleft'] < 0.30 and scan.region['fleft'] > 0.15:  
              speed.linear.x = 0.1    # go straight
              speed.angular.z = 0.0
            elif scan.region['left'] <= 0.15 and scan.region['fleft'] <= 0.15:
              speed.linear.x = 0.0    # adjust
              speed.angular.z = -0.1
            elif scan.region['fleft'] >= 0.35:
              speed.linear.x = 0.0
              speed.angular.z = 0.1   # turn corner
            else:
              speed.linear.x = 0.1    # else just go do something
              speed.angular.z = 0.0
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
