#!/usr/bin/env python

# Liz Miller
# 11.10.14
# lab2.py

# ROS python declaration
import rospy, tf
import math

# Imports for each of the message types used
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

################## GLOBALS ##################
global pub
global pose
pose = Pose()
global odom_tf
global odom_list

# executeTrajectory:
# parameters
# none
def executeTrajectory():
    pass

# spinWheels:
# parameters
# u1 - left wheel velocity
# u2 - right wheel velocity
# time - time
def spinWheels(u1, u2, time):
   print "spinWheels"
   #local vars
   radius = 0.05 # measured wheel radius
   b_dist = 0.30 # linear distance between wheels   
   
   #run at 10hz
   r = rospy.Rate(10)

   i = 0.0 # init counter
   while i < time: # run wheels until set time is reached
   
    # create a Twist message
    twist = Twist() 
    # translation component
    twist.linear.x = (radius/2)*(u1+u2)
    twist.linear.y = 0
    twist.linear.z = 0
   
    # rotation component
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = (radius/b_dist)*(u1-u2)
    
    # publish the twist
    pub.publish(twist)
    
    # sleep for 0.2 second
    rospy.sleep(0.2)
    
    # count each iteration
    i = i + 0.2

# Use this object to get the robot's Odometry
    odom_list = tf.TransformListener()
    
# Use this command to make the program wait for some seconds
   rospy.sleep(rospy.Duration(1, 0))
   
# driveStraight:
# parameters
# speed - speed (m/s)
# distance - distance (m)	
# This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    print "driveStraight"
    # starting config
    x_0 = pose.position.x
    y_0 = pose.position.y
    
    # x-distance travelled squared
    xdist = (pose.position.x - x_0)**2 
    # y-distance travelled squared
    ydist = (pose.position.y - y_0)**2
    
    #run at 10hz
    r = rospy.Rate(10)
     
    # keep driving until given distance is reached
    while xdist + ydist < distance**2 and not rospy.is_shutdown():
        
        # update pose reading
        x_0 = pose.position.x
        y_0 = pose.position.y
        
        # create a Twist message
        twist = Twist() 
        
        # translation component
        twist.linear.x = speed
        twist.linear.y = 0
        twist.linear.z = 0
   
        # rotation component
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        # publish the twist
        pub.publish(twist)
    
        # sleep for 0.2 second
        rospy.sleep(0.2)
        

def quatmsg_euler():
    global pose
    print pose
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    
    return yaw       
    
# rotate:
# parameters
# angle - angle (degrees) - converted to radians in body
# Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    print "rotate"
    angle = math.radians(angle) # convert from deg to rad

    #run at 10hz
    r = rospy.Rate(10)
    
    #define start angle
    ang_0 = quatmsg_euler()
    des_ang = ang_0 + angle
    print des_ang
    curr_ang = ang_0
     
    while curr_ang < des_ang and not rospy.is_shutdown():
        
        curr_ang = quatmsg_euler() # update current angle each iteration
        print curr_ang
        
        # create a Twist message
        twist = Twist() 
        
        # translation component
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
   
        # rotation component
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.2
        
        # publish the twist
        pub.publish(twist)
    
        # sleep for 0.2 second
        rospy.sleep(0.2)
        
    pub.publish(Twist()) # stop moving when angle is reached
    
# driveArc:
# parameters
# radius - radius (m)
# speed - speed (m/s)
# angle - angle (degrees) - converted to radians in body.
#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass

################## CALLBACK FUNCTIONS ##################

#Odometry Callback function.
def read_odometry(msg):
	global pose
	pose = msg.pose.pose
     
#Bumper Event Callback function
def readBumper(msg):
    pass    
        
# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented

# Main Function
if __name__ == '__main__':
    global pub
    rospy.init_node('emiller_lab2')
    sub = rospy.Subscriber("odom",Odometry,read_odometry)
    
    # Publisher for commanding robot motion
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist)
    
    #spinWheels(4,-4,5)
    driveStraight(0.5,.5)
    
    #rotate(90)
    
    # sleep for 1 second
    rospy.sleep(1)
    
    

    
 
   
    



