/*
 * Course To Vel
 * Purpose: Implements a course (speed, yaw) controller for the WamV
 * Inputs:
 *  - Speed: Desired speed
 *  - Yaw: Desired heading
 * Outputs: Twist message
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

            set_speed_ = 0.;
            set_yaw_ = 0.;

            received_course_ = false;

            heartbeat_timer_ = 0.;
            heartbeat_limit_ = 1.0;
            time_prev_ = ros::Time::now().toSec();


            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("/odometry/filtered", 1, &CourseToVel::localisationCallback, this);
            courseSub_ =  n_.subscribe("/cmd_course", 1, &CourseToVel::courseCallback, this);
    }
    // Used to get the rotation (change?)
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& ekf)
	{
		if(ros::ok() && received_course_ && heartbeat_timer_ < heartbeat_limit_)
		{
            double time_now = ros::Time::now().toSec();
            double time_diff;
            time_diff = time_now - time_prev_;
            heartbeat_timer_ = heartbeat_timer_ + time_diff;
            time_prev_ = time_now;

            tf::Quaternion quat;
            quat.setValue(ekf->pose.pose.orientation.x, ekf->pose.pose.orientation.y, ekf->pose.pose.orientation.z, ekf->pose.pose.orientation.w);

            // the tf::Quaternion has a method to access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            phi_ = yaw;

            geometry_msgs::Twist twist;
            double phid;
            // Calculate the difference between the desired and actual values
            phid = CourseToVel::constrainAngle(set_yaw_ - phi_);

            // std::cout << "Yaw: " << phi_ << "Diff: " << phid<< std::endl;
            // Fill out the twist message
            twist.linear.x = set_speed_;
            twist.linear.y = 0.;
            twist.linear.z = 0.;
            twist.angular.x = 0.;
            twist.angular.y = 0.;
            twist.angular.z = kp_*phid;
            velPub_.publish(twist);
            // std::cout << "Speed: " << twist.linear.x << " YawRate: " << twist.angular.z << std::endl;
		}
    }
    // Keeps the angle between -pi, pi
    float constrainAngle(float x)
    {
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }
    // Receive the course message and store it
    void courseCallback(const rowbot_msgs::Course::ConstPtr& msg)
    {
        if(ros::ok() && !received_odom_)
        {
            heartbeat_timer_ = 0.; //Reset the timer
            received_course_ = true;
            set_speed_ = msg->speed;
            set_yaw_ = msg->yaw;
        }
    }

private:
	ros::NodeHandle n_;
    ros::Subscriber subLocalise_;
    ros::Subscriber courseSub_;
    ros::Publisher velPub_;

	double set_speed_; // Not currently used but could be useful.

    double phi_;

    double set_yaw_;

    double kp_;

    bool received_odom_;
    bool received_course_;

    double heartbeat_timer_;
    double heartbeat_limit_;
    double time_prev_;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "course_to_twist");

  CourseToVel c2v;

  ros::spin();


  return 0;
}
