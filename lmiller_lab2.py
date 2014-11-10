#!/usr/bin/env python

# ROS python declaration
import rospy, tf
import math

# Imports for each of the message types used
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# These are global variables. Write "global <variable_name>" in any other function
# to gain access to these global variables
    
global pub
global pose
global odom_tf
global odom_list

sub = rospy.Subscriber("/cmd_vel_mux/input/teleop",Twist,read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    
bumper_sub = rospy.Subscriber('...', ..., readBumper, queue_size=1) # Callback function to handle bumper events
    
# Use this object to get the robot's Odometryn
    odom_list = tf.TransformListener()
    
# Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

#This function sequentially calls methods to perform a trajectory.
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

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    pass  # Delete this 'pass' once implemented

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
   pass  # Delete this 'pass' once implemented

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    angle=angle*radians; # convert to radians
    pass  # Delete this 'pass' once implemented

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented

################## CALLBACK FUNCTIONS ##################

#Odometry Callback function.
def read_odometry(msg):
    # sign that callback works
    rospy.loginfo("callback recvd")
    # linear message
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    # angular message
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    
#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
    # message that bumper has been pressed
    rospy.loginfo("bumper triggered")
    
    # setup the subscriber
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, readBumper)
        
# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented


# Main Function
if __name__ == '__main__':
   
    pub = rospy.Publisher("/mobile_base/commands/velocity", Twist) # Publisher for commanding robot motion
    rospy.init_node('lmiller-lab2')
    
    print "Starting Lab 2"
        
    # Subscribe to turtlebot

    print "Lab 2 complete!"
