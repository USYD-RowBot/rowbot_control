#!/usr/bin/env python
# Author: Jackson Shields


import sys
import argparse
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point
from std_msgs.msg import Float64, Bool, Float32
import math
import tf
from rospy.numpy_msg import numpy_msg

from rowbot_msgs.msg import Course

from path_following import NLGLvtp, purepursuit

class WamNav():
    """
    This class navigates the WamV in the uuv_simulator
    """

    def __init__(self):
        """Initiates the navigator class, with a user specified speed,
        following type and waypoint hitting tolerance

        Args:
            speed (float): The forwards velocity of the vessel in m/s
            use_nlgl (bool): Whether to use Non-Linear Guidance Law path following
            tolerance (float): The tolerance to hit a waypoint (if the distance
            to the waypoint is less, the waypoint will be considered hit.)

        Returns:
            type: None

        """
        robot_name = rospy.get_param("~robot_name", "")
        # # Map the namespace into the topics
        if robot_name is None or robot_name == "":
            course_topic = '/cmd_course'
            request_topic = '/request_waypoints'
            odom_topic = '/odometry/filtered'
            waypoint_topic = '/waypoints'
        else:
            course_topic = '/' + robot_name + '/cmd_course'
            request_topic = '/' + robot_name + '/request_waypoints'
            odom_topic = '/' + robot_name + '/odometry/filtered'
            waypoint_topic = '/' + robot_name + '/waypoints'
        # The publishers
        self.wpRequestPub_ = rospy.Publisher(request_topic, Bool, queue_size=1)
        self.coursePub_ = rospy.Publisher(course_topic, Course, queue_size=1)
        # Go - if go is 0, don't move, if 1, then go.
        self.go = False
        # Pose
        self.px = 0.
        self.py = 0.
        self.phi = 0.
        # Waypoints
        self.wx = 0.
        self.wy = 0.
        self.tx = 0.
        self.ty = 0.
        # These are kept in self for logging (not really necessary)
        self.distwp = 0.
        self.wpAngle = 0.
        # Holds the waypoints list
        self.waypoints = []
        self.wcount = 0.
        self.wIdx = 0.
        # Tolerance to hit the waypoints
        self.speed = rospy.get_param("~speed", 1.0)
        self.use_nlgl = rospy.get_param("~use_nlgl", True)
        self.tolerance = rospy.get_param("~tolerance", 4.0)
        # Prefer adding the subscribers at the end of init
        rospy.Subscriber(odom_topic, Odometry, self.localisationCallback)
        rospy.Subscriber(waypoint_topic, PoseArray, self.waypointCallback)

        print("End of Init")


    def localisationCallback(self, data):
        """ Receives the localisation data and navigates the vessel

        Args:
            data (Odometry): An odometry message

        Returns:
            None

        """
        if self.go:
            self.px = data.pose.pose.position.x
            self.py = data.pose.pose.position.y
            quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            self.phi = euler[2]

            self.waypointSelect(self.tolerance)
            self.wpAngle = math.atan2(self.ty - self.wy, self.tx - self.wx)

            if self.use_nlgl:
                [xt, yt] = NLGLvtp(self.wx, self.wy, self.tx, self.ty, self.px, self.py, 10)
                #print xt, yt
                phid = purepursuit(xt, yt, self.px, self.py)
            else:
                phid = purepursuit(self.tx, self.ty, self.px, self.py)

            # phid is the angle to the target waypoint
            # Stands for 'phi desired'
            course_msg = Course()
            course_msg.speed = self.speed
            course_msg.yaw = phid
            self.coursePub_.publish(course_msg)

    def waypointCallback(self, pose_array):
        """ Receives the waypoint array from the waypoint publisher

        Args:
            pose_array (PoseArray): A 6D pose array of the waypoints (only x,y are used)

        Returns:
            None

        """
        # Delete all the old waypoints
        self.waypoints = []
        # Append the current position
        self.waypoints.append(np.array([self.px, self.py]))
        for pose in pose_array.poses:
            self.waypoints.append(np.array([pose.position.x, pose.position.y]))
        self.wIdx = 0
        self.wcount = len(self.waypoints)
        # Set the targets here - avoids it hitting waypoint select with the wrong targets
        self.tx = self.waypoints[self.wIdx + 1][0]
        self.ty = self.waypoints[self.wIdx + 1][1]
        self.go = True
        print("Received waypoints - commencing")

    def requestWaypoints(self):
        """ Publishes a message saying the current waypoints are finished,
        and more are needed.

        Returns:
            None

        """
        self.go = False
        course_msg = Course()
        course_msg.speed = 0.0
        course_msg.yaw = self.phi # Keep the current phi
        self.coursePub_.publish(course_msg)
        req = Bool()
        req.data = True
        self.wpRequestPub_.publish(req)
        print("Requesting new waypoints")


    def waypointSelect(self, tolerance):
        """ Tests if the waypoint has been hit, then selects the waypoint

        Args:
            tolerance (float): If the distance to the waypoint is less than this,
            it is considered hit

        Returns:
            None

        """
        self.dist2wp = math.sqrt((self.ty - self.py)**2 + (self.tx - self.px)**2)
        #print dist2wp
        if(self.dist2wp < tolerance):
            print("Waypoint hit @ x:%f y:%f with x:%f y:%f" %(self.tx, self.ty, self.px, self.py))
            self.wIdx = self.wIdx + 1
            print("Index: %d Count: %d" %(self.wIdx, self.wcount))
            if(self.wIdx >= (self.wcount-1)):
                self.requestWaypoints()

                return

        self.tx = self.waypoints[self.wIdx + 1][0]
        self.ty = self.waypoints[self.wIdx + 1][1]
        self.wx = self.waypoints[self.wIdx][0]
        self.wy = self.waypoints[self.wIdx][1]


if __name__ == '__main__':
    """
    Navigates the WamV to a set of waypoints
    """

    rospy.init_node('wamv_navigator', anonymous=True)

    hnav = WamNav()

    rospy.spin()
