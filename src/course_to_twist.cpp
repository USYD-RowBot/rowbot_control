/*
 * Course To Vel
 * Purpose: Implements a course (speed, yaw) controller for the WamV
 * Inputs:
 *  - Speed: Desired speed
 *  - Yaw: Desired heading
 * Outputs: Drive Messages which steer the WamV
*/
#include <math.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "rowbot_msgs/Course.h"

class CourseToVel
{
public:
    CourseToVel()
	{
            // ros::NodeHandle n_("~");
            // Course Publish
            velPub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            // Get the relevant parameters
            ros::param::get("~course_kp", kp_);
            //kp_ = 0.5;

            received_odom_ = false; // Shouldn't use course if no odom received

            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("/odometry/filtered", 1, &CourseToVel::localisationCallback, this);
            courseSub_ =  n_.subscribe("/cmd_course", 1, &CourseToVel::courseCallback, this);
    }
    // Used to get the rotation (change?)
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& ekf)
	{
		if(ros::ok())
		{
            tf::Quaternion quat;
            quat.setValue(ekf->pose.pose.orientation.x, ekf->pose.pose.orientation.y, ekf->pose.pose.orientation.z, ekf->pose.pose.orientation.w);

            // the tf::Quaternion has a method to access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            phi_ = yaw;
		}
    }
    float constrainAngle(float x)
    {
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }
    // Receive the course message, and publish a drive message
    void courseCallback(const rowbot_msgs::Course::ConstPtr& msg)
    {
        if(ros::ok() && !received_odom_)
        {
            float phid, setpoint;
            geometry_msgs::Twist twist;
            // Calculate the difference between the desired and actual values
            phid = msg->yaw - phi_;
            phid = CourseToVel::constrainAngle(phid);
            // std::cout << "Yaw: " << phi_ << "Diff: " << phid<< std::endl;
            // Fill out the twist message
            twist.linear.x = msg->speed;
            twist.linear.y = 0.;
            twist.linear.z = 0.;
            twist.angular.x = 0.;
            twist.angular.y = 0.;
            twist.angular.z = kp_*phid;

            velPub_.publish(twist);
        }
    }

private:
	ros::NodeHandle n_;
    ros::Subscriber subLocalise_;
    ros::Subscriber courseSub_;
    ros::Publisher velPub_;

	double speed_; // Not currently used but could be useful.

    double phi_;

    double kp_;

    bool received_odom_;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "course_to_twist");

  CourseToVel c2v;

  ros::spin();


  return 0;
}
