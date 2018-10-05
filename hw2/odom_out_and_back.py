#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        self.range_ahead = 1
        self.range_right = 1

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.scan = rospy.Subscriber('scan', LaserScan, scan_callback)
        
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        linear_speed = 0.15
        
        # Set the travel distance in meters
        goal_distance = 10.0

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)
        
        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = 0

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        position = Point()
        # Get the starting position values     
        (position, rotation) = self.get_odom()

        x_start = position.x
        y_start = position.y

        goal_x = 10
        goal_y = 0

        obstacle_x = 0
        obstacle_y = 0

        # Initialize the movement command
        move_cmd = Twist()
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        

        # Loop until we reach the goal
        while (position.x != goal_x and position.y != goal_y and not rospy.is_shutdown()):
            
            # Initialize the movement command
            move_cmd = Twist()
            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            r.sleep()

            # Get new position and rotation
            (position, rotation) = self.get_odom()
            
            # Check if robot encountered an obstacle
            if (self.range_ahead < 0.8):
                # Store coordinates of encounter
                obstacle_x = position.x
                obstacle_y = position.y

                # Turn left until obstacle cannot be detected
                while (self.range_right < 10):
                    # Initialize the movement command
                    move_cmd = Twist()
                    # Publish the Twist message and sleep 1 cycle         
                    move_cmd.angular.z = angular_speed
                    self.cmd_vel.publish(move_cmd)
                    
                    r.sleep()

                # Trace the contour
                tracing = True
                while (tracing):
                    # Initialize the movement command
                    move_cmd = Twist()
                    move_cmd.linear.x = linear_speed
                    self.cmd_vel.publish(move_cmd)
                    
                    r.sleep()

                    # On m-line
                    if (position.y == goal_y):
                        tracing = False
                    # No solution
                    elif (position.x == obstacle_x and position.y == obstacle_y):
                        tracing = False
                        rospy.loginfo("No solution possible.")
                        self.shutdown()

                # Rotate robot until oriented at 0 degrees
                move_cmd = Twist()

                # Set the movement command to a rotation
                move_cmd.angular.z = angular_speed
                
                # Track the last angle measured
                last_angle = rotation
                
                # Track how far we have turned
                turn_angle = 0
                    
                while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                    # Publish the Twist message and sleep 1 cycle         
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                    
                    # Get the current rotation
                    (position, rotation) = self.get_odom()
                    
                    # Compute the amount of rotation since the last loop
                    delta_angle = normalize_angle(rotation - last_angle)
                    
                    # Add to the running total
                    turn_angle += delta_angle
                    last_angle = rotation
                # # Stop the robot before the next leg
                # move_cmd = Twist()
                # self.cmd_vel.publish(move_cmd)
                # rospy.sleep(1)
            
        # Stop the robot for good
        self.cmd_vel.publish(Twist())
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def scan_callback(self, msg):
        self.range_ahead = min(msg.ranges)
        self.range_right = msg.ranges[len(msg.ranges)]
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

