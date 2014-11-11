#!/usr/bin/env python

# Liz Miller
# 11.10.14
# lab2.py

# ROS python declaration
import rospy, tf
import math

# Imports for each of the message types used
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

################## GLOBALS ##################
global pub
global pose
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
  
   # Publisher for commanding robot motion
   pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist)

   i = 0 # init counter
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
    
    # sleep for 1 second
    rospy.sleep(1)
    
    # count each iteration
    i = i + 1

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
    print "driveStraight"
    # starting config
    x0 = pose.position.x
    y0 = pose.position.y
    
    # x-distance travelled squared
    xdist = (pose.position.x - x0)**2 
    # y-distance travelled squared
    ydist = (pose.position.y - y0)**2
    
    #run at 10hz
    r = rospy.Rate(10)
     
    # keep driving until given distance is reached
    while xdist + ydist < distance**2:
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
    
        # sleep for 1 second
        rospy.sleep(1)
    
# rotate:
# parameters
# angle - angle (degrees) - converted to radians in body
# Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    print "rotate"
    pass
    
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
	pass
     
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
    rospy.init_node('emiller_lab2')
    spinWheels(3,3,15)
    # sleep for 1 second
    rospy.sleep(1)


    
 
   
    



