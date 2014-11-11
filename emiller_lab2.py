#!/usr/bin/env python

# Liz Miller
# 11.10.14
# emiller_lab2.py

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

# Turtlebot measurements
global	b = .25 # linear distance (m) between both wheels on turtlebot
global	r = .05	# wheel radii (m)
  
# Use this object to get the robot's Odometry
    odom_list = tf.TransformListener()
    
# Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

# executeTrajectory:
# parameters
# none
def executeTrajectory():
    print "drive fwd .60m"
    driveStraight(0.1,0.60)
    driveStraight(0,0) # stop wheels
    
    print "r-turn 90-deg"
    rotate(90)
    
    print "drive -180 arc, .15m radius"
    driveArc(-180,0.1,0.15)

    print "l-turn 135-deg"
    rotate(135)
    
    print "drive fwd .42m"
    driveStraight(0.1,0.42)

# spinWheels:
# parameters
# u1 - left wheel velocity
# u2 - right wheel velocity
# time - time
def spinWheels(u1, u2, time):
   twist = Twist() # create a message Twist
   pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist) # Publisher for commanding robot motion
  count = 0
  while (count < time): # run wheels until set time is reached
   # publish translation to twist
   twist.linear.x = (r/2)*(u1+u2)
   twist.linear.y = 0
   twist.linear.z = 0
   
   # publish rotation to twist
   twist.angular.x = 0
   twist.angular.y = 0
   twist.angular.z = (r/b)*(u1-u2)

   # sleep for 1 second
   rospy.sleep(1)

   # count each iteration
   count = count + 1

# driveStraight:
# parameters
# speed - speed (m/s)
# distance - distance (m)	
# This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
   twist = Twist() # create a message Twist
   pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist) # Publisher for commanding robot motion
   # obtain distance
   current_dist = odom_caller.x
   final = odom_ + distance
   while (current_dist < distance):	
   # publish translation to twist
   twist.linear.x = (r/2)*(speed+speed)
   twist.linear.y = 0
   twist.linear.z = 0

# rotate:
# parameters
# angle - angle (degrees) - converted to radians in body
# Accepts an angle and makes the robot rotate around it.
def rotate(angle):
   twist = Twist() # create a message Twist
   pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist) # Publisher for commanding robot motion
   angle = math.radians(angle) # convert to radians
   while True:
   # publish translation to twist
   twist.linear.x = 0
   twist.linear.y = 0
   twist.linear.z = 0

   # publish rotation to twist
   twist.angular.x = radius + speed*math.cos(angle)
   twist.angular.y = radius + speed*math.sin(angle)
   twist.angular.z = angle
    
# driveArc:
# parameters
# radius - radius (m)
# speed - speed (m/s)
# angle - angle (degrees) - converted to radians in body.
#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
   twist = Twist() # create a message Twist
   pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist) # Publisher for commanding robot motion
   angle = math.radians(angle) # convert to radians
   # publish rotation to twist
   twist.angular.x = radius + speed*math.cos(angle)
   twist.angular.y = radius + speed*math.sin(angle)
   twist.angular.z = angle

################## CALLBACK FUNCTIONS ##################

#Odometry Callback function.
def read_odometry(msg):
	odom_caller 
   
    
#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
    # message that bumper has been pressed
    rospy.loginfo("bumper triggered",msg.msg)
    
    # setup the subscriber
    bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, readBumper) # Callback function to handle bumper events
        
# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented


# Main Function
if __name__ == '__main__':
    rospy.init_node('emiller_lab2')
 

odom_sub = rospy.Subscriber("",Twist, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
 # sleep for 1 second
   rospy.sleep(1)


    
 
   
    




