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
global odom_tf
global odom_list
global bumperPressed
   
# spinWheels:
# parameters
# u1 - left wheel velocity
# u2 - right wheel velocity
# time - time
def spinWheels(u1, u2, time):
   print "spinWheels"
   global pub
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
  
# driveStraight:
# parameters
# speed - speed (m/s)
# distance - distance (m)	
# This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    global pub
    print "driveStraight"
    
    # starting config
    x_0 = pose.position.x
    y_0 = pose.position.y
    
    print x_0
    print y_0
     
    # keep driving until given distance is reached
    while (pose.position.x-x_0)**2+(pose.position.y-y_0)**2 < distance**2 and not rospy.is_shutdown():
    
        print x_0
        print y_0
        
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


# rotate:
# parameters
# angle - angle (radians)
# Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    print "rotate"
    
    global pub
    global yaw
        
    # set rate 50Hz
    r = rospy.Rate(50)
    
    # initialize data
    th0 = yaw # starting angle
    thf = yaw + angle # final angle
    
    # convert start/stop to accept any orientation
    if thf > math.pi: # final angle > pi (180 deg)
        thf = thf-2*math.pi # adjust final angle to final angle - 360 deg
    elif thf < -math.pi:  # final angle < -pi (-180 deg)
        thf = thf+2*math.pi # adjust final angle to final angle + 360 deg
    
    # consider positive/negative input angles
    if (angle < 0): # given angle is negative
        speed = -1 # rotate negative (opposite) direction
    else:
        speed = 1 # rotate in positive (same) direction
     
    while abs(yaw-thf) > 0.025 and not rospy.is_shutdown():
        
        # create a Twist message
        twist = Twist() 
        
        # translation component
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
   
        # rotation component
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = speed
        
        # publish the twist
        pub.publish(twist)
        r.sleep()     
           
# driveArc:
# parameters
# radius - radius (m)
# speed - speed (m/s)
# angle - angle (degrees) - converted to radians in body.
#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    print "driveArc"
    global pub
    global yaw
    
    # set rate 50Hz
    r = rospy.Rate(50)
    
    # initialize data
    th0 = yaw # starting angle
    thf = yaw + angle # final angle
    
    # convert start/stop to accept any orientation
    if thf > math.pi: # final angle > pi (180 deg)
        thf = thf-2*math.pi # adjust final angle to final angle - 360 deg
    elif thf < -math.pi:  # final angle < -pi (-180 deg)
        thf = thf+2*math.pi # adjust final angle to final angle + 360 deg
    
    while abs(yaw-thf) > 0.025 and not rospy.is_shutdown():
        # create a Twist message
        twist = Twist() 
        # translation component
        twist.linear.x = speed
        twist.linear.y = 0
        twist.linear.z = 0
        # rotation component
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = speed/radius
        # publish the twist
        pub.publish(twist)
        r.sleep()
        
# executeTrajectory:
# parameters
# none
def executeTrajectory():
    print "executeTrajectory"
    while not rospy.is_shutdown() and True: #forever while
        if bumperPressed == 0: # if pressed
            pass # do nothing
        elif bumperPressed == 1: # otherwise execute trajectory
            driveStraight(.5, .60) # drive fwd 60cm
            rospy.sleep(1.0)
            rotate(1.57) # turn 90-deg
            rospy.sleep(1.0)
            driveArc(0.15, .1, -3.14) # make an -180-deg arc w/ 15cm radius
            rospy.sleep(1.0)
            rotate(2.36) # turn left 135-deg
            rospy.sleep(1.0)
            driveStraight(.5, .42) # drive fwd 42cm
            
################## CALLBACK FUNCTIONS ##################

#Odometry Callback function.
def read_odometry(msg):
    global pose
    global yaw
    pose = msg.pose.pose
    
    #code from http://answers.ros.org/question/69754/quaternion-transformations-in-python/    
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
     
#Bumper Event Callback function
def readBumper(msg):
    global bumperPressed
    if (msg.state == 1):
        print "bumper pressed"
        bumperPressed = 1   

################## MAIN ##############################
if __name__ == '__main__':
    global pub
    global yaw
    global pose
    global bumperPressed
    global odom_list
    global odom_tf
    rospy.init_node('emiller_lab2')
    
    # init bumper to 'unpressed'
    bumperPressed = 0
    
    # Publisher for commanding robot motion
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist)
    
    # Callback function to read in robot Odometry messages
    sub = rospy.Subscriber("odom",Odometry,read_odometry, queue_size=1)
    
    # Callback function to handle bumper events
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) 
       
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(2.0)
    
    # Function calls: remove comment (#) and X.
    
    # 1. spinWheels(4,-4,5)
    # 2. driveStraight(0.5,0.5)
    # 3. rotate(1.5)
    # 4. driveArc(.5,.1,1.5)
    # 5. 
    executeTrajectory()    
    
    # sleep for 1 second
    rospy.sleep(1)
