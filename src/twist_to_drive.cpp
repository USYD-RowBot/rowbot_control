/*
 * Twist to Drive
 * Purpose: Implements a velocity controller for the WAM-V
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

class TwistToDrive
{
public:
    TwistToDrive()
	{
            // ros::NodeHandle n_("~");
            // Course Publish
            stbdPub_ = n_.advertise<std_msgs::Float32>("/right_thrust_cmd", 1);
            portPub_ = n_.advertise<std_msgs::Float32>("/left_thrust_cmd", 1);
            // Get the relevant parameters
            // ros::param::get("~lin_kp", lin_kp_);
            // ros::param::get("~lin_ki", lin_ki_);
            // ros::param::get("~lin_kd", lin_kd_);
            // ros::param::get("~ang_kp", ang_kp_);
            // ros::param::get("~ang_ki", ang_ki_);
            // ros::param::get("~ang_kd", ang_kd_);
            //kp_ = 0.5;
            ang_kp_ = 0.5;

            lin_kp_ = 1.0;
            lin_ki_ = 1.0;

            heartbeat_timer_ = 0.;
            heartbeat_limit_ = 1.0;
            time_prev_ = ros::Time::now().toSec();

            received_odom_ = false; // Shouldn't use course if no odom received

            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("/odometry/filtered", 1, &TwistToDrive::localisationCallback, this);
            velSub_ =  n_.subscribe("/cmd_vel", 1, &TwistToDrive::velCallback, this);

            ROS_DEBUG_STREAM("END OF INIT");

    }
    // Used to get the rotation (change?)
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& odom)
	{
		if(ros::ok())
		{
            double time_now = ros::Time::now().toSec();
            double time_diff;
            time_diff = time_now - time_prev_;
            heartbeat_timer_ = heartbeat_timer_ + time_diff;
            time_prev_ = time_now;


            tf::Quaternion quat;
            quat.setValue(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            // the tf::Quaternion has a method to access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            phi_ = yaw;

            // Get the received twist message
            geometry_msgs::Twist twist;
            twist = odom->twist.twist;

            double alpha, velU, delta;

            // Yaw Rate Controller
            alpha = twistSet_.angular.z - twist.angular.z;
            // TODO Implement Speed Controller here
            // Find forwards and sidewards velocity
            velU = sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2));
            delta = twistSet_.linear.x - velU;

            double Ts, Tp, Vs, Vp, vel_offset;
            // Velocity Offset = Dist Between Hulls * Angular Velocity Command
            vel_offset = 2.4384 * (twistSet_.angular.z + alpha*ang_kp_);
            // Change linear.x to linear.x + delta?
            Vs = (twistSet_.linear.x+delta*lin_kp_) + vel_offset;
            Vp = (twistSet_.linear.x+delta*lin_kp_) - vel_offset;

            // std::cout << "Vs: " << Vs << " Vp: " << Vp << std::endl;

            // TODO Account for saturation - not complete
            double saturation_cmd_pos, saturation_cmd_neg, saturation_vel_pos, saturation_vel_neg;
            saturation_cmd_pos = 1.0;
            saturation_cmd_neg = -1.0;
            saturation_vel_pos = command_to_vel(saturation_cmd_pos, 0.0, 0.31, 5.71, 0.65, 0.27, 0.18);
            saturation_vel_neg = command_to_vel(saturation_cmd_neg, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
            bool within_saturation_limits = false;

            // Sort the saturation, while prioritising the yaw rate over speed.
            while (!within_saturation_limits)
            {
                // Do backwards first
                if (Vs < saturation_vel_neg)
                {
                    Vs = saturation_vel_neg+0.01;
                    Vp = Vs - 2*vel_offset;
                }
                if (Vp < saturation_vel_neg)
                {
                    Vp = saturation_vel_neg+0.01;
                    Vs = Vp + 2*vel_offset;
                }
                if (Vs > saturation_vel_pos)
                {
                    Vs = saturation_vel_pos-0.01;
                    Vp = Vs - 2*vel_offset;
                }
                if (Vp > saturation_vel_pos)
                {
                    Vp = saturation_vel_pos-0.01;
                    Vs = Vp + 2*vel_offset;
                }
                if(Vs > saturation_vel_neg && Vs < saturation_vel_pos && Vp > saturation_vel_neg && Vs < saturation_vel_pos)
                {
                    within_saturation_limits = true;
                }
                else // If still not within limits - call the while loop off FIXME
                {
                    if (Vs < saturation_vel_neg)
                    {
                        Vs = saturation_vel_neg+0.01;
                    }
                    if (Vp < saturation_vel_neg)
                    {
                        Vp = saturation_vel_neg+0.01;
                    }
                    if (Vs > saturation_vel_pos)
                    {
                        Vs = saturation_vel_pos-0.01;
                    }
                    if (Vp > saturation_vel_pos)
                    {
                        Vp = saturation_vel_pos-0.01;
                    }
                    within_saturation_limits = true;
                }
            }
            // Convert velocity to thrust command - linearising the output
            if(Vs >= 0) // Use positive coefficients
            {
                // GLF+ Coefficients: A=0,K=0.31,B=5.71,v=0.65,C=0.27,M=0.18
                Ts = vel_to_command(Vs, 0, 0.31, 5.71, 0.65, 0.27, 0.18);
            }
            else // Use negative coefficients
            {
                // GLF- Coefficients: A=-1.35,K=0,B=4.89,v=0.05,C=1.00,M=-1.31
                Ts = vel_to_command(Vs, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
            }
            if(Vp >= 0)
            {
                // GLF+ Coefficients: A=0,K=0.31,B=5.71,v=0.65,C=0.27,M=0.18
                Tp = vel_to_command(Vp, 0, 0.31, 5.71, 0.65, 0.27, 0.18);
            }
            else
            {
                // GLF- Coefficients: A=-1.35,K=0,B=4.89,v=0.05,C=1.00,M=-1.31
                Tp = vel_to_command(Vp, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
            }

            // std::cout << "Ts: " << Ts << " Tp: " << Tp << std::endl;

            std_msgs::Float32 ts_msg;
            std_msgs::Float32 tp_msg;
            ts_msg.data = float(Ts);
            tp_msg.data = float(Tp);
            // ROS_DEBUG_STREAM("Ts: " << Ts << " Tp: " << Tp);
            stbdPub_.publish(ts_msg);
            portPub_.publish(tp_msg);
		}
    }
    // Command to Thrust using GLF (Generalised Logistic Functions)
    float command_to_vel(float command, float A, float K, float B, float v, float C, float M)
    {
        // Uses GLF to map command to thrust
        float vel;
        vel = A + (K-A)/pow((C+exp(-B*(command -M))),1/v);
        return vel;
    }
    // Velocity to Command
    float vel_to_command(float vel, float A, float K, float B, float v, float C, float M)
    {
        // Equation:
        // command = \frac{ln(\frac{K-A}{vel-A}^v - C)}{-B} + M
        // Uses a rearranged GLF to map command to thrust
        float command;
        command = log((pow((K-A)/(vel - A),v) - C))/(-B) + M;
        return command;
    }
    // Keeps the angle between -pi, pi
    float constrainAngle(float x)
    {
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }
    // Receive the course message, and publish a drive message
    void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        if(ros::ok())
        {
            twistSet_ = *msg;
            ROS_DEBUG_STREAM("Twist received");
        }
    }

private:
	ros::NodeHandle n_;
    ros::Subscriber subLocalise_;
    ros::Subscriber velSub_;
    ros::Publisher stbdPub_;
    ros::Publisher portPub_;

    double phi_;

    double lin_kp_;
    double lin_kd_;
    double lin_ki_;

    double ang_kp_;
    double ang_kd_;
    double ang_ki_;


    bool received_odom_;

    geometry_msgs::Twist twistSet_;

    double heartbeat_timer_;
    double heartbeat_limit_;
    double time_prev_;

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "twist_to_drive");

  TwistToDrive t2d;

  ros::spin();


  return 0;
}
