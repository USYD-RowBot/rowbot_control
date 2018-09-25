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

/*
TwistToDrive is a class that performs the velocity controller. It subscribes to
an odometry message, and a course command, and publishes to the motors.
 */
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
            ros::param::get("~lin_kp", lin_kp_);
            ros::param::get("~lin_ki", lin_ki_);
            ros::param::get("~lin_kd", lin_kd_);
            ros::param::get("~ang_kp", ang_kp_);
            ros::param::get("~ang_ki", ang_ki_);
            ros::param::get("~ang_kd", ang_kd_);
            // ang_kp_ = 0.5;
            // ang_ki_ = 0.0;
            // ang_kd_ = 0.0;
            //
            // lin_kp_ = 1.0;
            // lin_ki_ = 0.0;
            // lin_kd_ = 0.0;

            prev_lin_error_ = 0.0;
            acc_lin_error_ = 0.0;
            // max_lin_windup_ = 1.0;
            ros::param::get("~max_linear_windup", max_lin_windup_)
            ros::param::get("~max_angular_windup", max_ang_windup_)
            prev_ang_error_ = 0.0;
            acc_ang_error_ = 0.0;
            // max_ang_windup_ = 1.0;

            // The heartbeat is implemented to prevent the robot moving with an
            // out of date command. Default limmit is 1.0.
            heartbeat_timer_ = 0.;
            heartbeat_limit_ = 1.0;
            time_prev_ = ros::Time::now().toSec();

            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("/odometry/filtered", 1, &TwistToDrive::localisationCallback, this);
            velSub_ =  n_.subscribe("/cmd_vel", 1, &TwistToDrive::velCallback, this);

            ROS_DEBUG_STREAM("END OF INIT");

    }
    // localisationCallback:
    //  Purpose: The callback from the odometry. Used to find the error between the commmand and
    //           the actual error. Used as the location for the PID loop.
    //  Subscribes: Vehicle odometry (nav_msgs::Odometry)
    //  Publishes: Left Motor Command (Float32), Right Motor Command (Float32)
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& odom)
	{
		if(ros::ok() && heartbeat_timer_ < heartbeat_limit_)
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
            double ang_error, ang_effort;
            double lin_error, lin_effort;

            // Yaw Rate Controller
            ang_error = twistSet_.angular.z - twist.angular.z;

            // Find forwards and sidewards velocity
            velU = sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2));
            // TODO find sidewards velocity velV and find way to eliminate - at the moment NLGL the bulk of this.
            lin_error = twistSet_.linear.x - velU;

            // Calculate the iterms - do this separately to check for windup limits
            double ang_iterm, lin_iterm;
            ang_iterm = acc_ang_error_ + time_diff*ang_error;
            // Check the positive windup limit
            if(ang_iterm > max_ang_windup_)
            {
                ang_iterm = max_ang_windup_;
            }
            else if(ang_iterm < -max_ang_windup) // Check the negative windup limit
            {
                ang_iterm = -max_ang_windup_;
            }
            // Calculat the linear i-term
            lin_iterm = acc_lin_error_ + time_diff*lin_error;
            // Check positive windup limit
            if(lin_iterm > max_lin_windup_)
            {
                lin_iterm = max_lin_windup_;
            }
            else if(lin_iterm < -max_lin_windup_) // Check negative windup limit
            {
                lin_iterm = -max_lin_windup;
            }

            // If it is just the P-Controller
            ang_effort = ang_kp_*ang_error;
            lin_effort = lin_kp_*lin_error;

            // A basic PID controller
            // ang_effort = ang_kp_*ang_error + ang_ki_*ang_iterm + ang_kd_*(ang_error - prev_ang_error_)/time_diff;
            // lin_effort = lin_kp_*lin_error + lin_ki_*lin_iterm + lin_kd_*(lin_error - prev_lin_error_)/time_diff;

            // Store the errors into previous error, for use in the D-term.
            prev_ang_error_ = ang_error;
            prev_lin_error_ = lin_error;
            // Add the errors to the accumulated sums
            acc_lin_error_ = acc_lin_error_ + lin_error;
            acc_ang_error_ = acc_ang_error_ + ang_error;

            double Ts, Tp, Vs, Vp, vel_offset;
            // Velocity Offset = Dist Between Hulls * Angular Velocity Command
            vel_offset = 2.4384 * (twistSet_.angular.z + ang_effort);
            // Change linear.x to linear.x + delta?
            Vs = (twistSet_.linear.x+lin_effort) + vel_offset;
            Vp = (twistSet_.linear.x+lin_effort) - vel_offset;

            // std::cout << "Vs: " << Vs << " Vp: " << Vp << std::endl;

            // Saturation Considerations
            double saturation_cmd_pos, saturation_cmd_neg, saturation_vel_pos, saturation_vel_neg;
            // The maximum saturation command of the motors, both going forwards and backwards
            saturation_cmd_pos = 1.0;
            saturation_cmd_neg = -1.0;

            // This method uses a command-velocity mapping (skips the thrust bit - which is imperfect but works well for Sim)

            // Calculate the saturation velocity, the maximum velocity the motors can achieve at the command limits.
            // These values are obtained from experiments, using GLF.
            saturation_vel_pos = command_to_vel(saturation_cmd_pos, 0.0, 0.31, 5.71, 0.65, 0.27, 0.18);
            saturation_vel_neg = command_to_vel(saturation_cmd_neg, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
            // The motors shouldn't be allowed to operate when saturated - with the GLF mapping it can output NaNs.
            bool within_saturation_limits = false;

            // Sort the saturation, while prioritising the yaw rate over speed.
            while (!within_saturation_limits)
            {
                // Do the positive saturation velocities first - as these have higher saturation velocities
                if (Vs > saturation_vel_pos)
                {
                    // If it is over the saturation velocity, set to the maximum saturation value
                    // and adjust the other motor, so that the desired yaw_rate is obtained.
                    Vs = saturation_vel_pos-0.01;
                    Vp = Vs - 2*vel_offset;
                }
                if (Vp > saturation_vel_pos)
                {
                    // If it is over the saturation velocity, set to the maximum saturation value
                    // and adjust the other motor, so that the desired yaw_rate is obtained.
                    Vp = saturation_vel_pos-0.01;
                    Vs = Vp + 2*vel_offset;
                }
                // Do backwards last - as this has a smaller saturation value
                if (Vs < saturation_vel_neg)
                {
                    // If the value is below the maximum backwards velocity, set the maximum backwards velocity.
                    Vs = saturation_vel_neg+0.01;
                    // TODO check this.
                    if (vel_offset < saturation_vel_neg)
                    {
                        Vp = abs(Vs);
                    }
                    else
                    {
                        Vp = Vs - 2*vel_offset;
                    }
                }
                if (Vp < saturation_vel_neg)
                {
                    // If the value is below the maximum backwards velocity, set the maximum backwards velocity.
                    Vp = saturation_vel_neg+0.01;
                    if (vel_offset < saturation_vel_neg)
                    {
                        Vs = abs(Vp);
                    }
                    else
                    {
                        Vs = Vp + 2*vel_offset;
                    }
                }
                // Checks if the velocities are within targets
                if(Vs > saturation_vel_neg && Vs < saturation_vel_pos && Vp > saturation_vel_neg && Vs < saturation_vel_pos)
                {
                    within_saturation_limits = true;
                }
                else // If still not within limits - call the while loop off FIXME
                {
                    // It shouldn't get here, this is a last resort.
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

            // Create the output messages
            std_msgs::Float32 ts_msg;
            std_msgs::Float32 tp_msg;
            // Convert to float (from double)
            ts_msg.data = float(Ts);
            tp_msg.data = float(Tp);
            // ROS_DEBUG_STREAM("Ts: " << Ts << " Tp: " << Tp);
            // Publish to the motors
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
            // Resets the heartbeat timer
            heartbeat_timer_ = 0.0;
            // Saves the command message
            twistSet_ = *msg;
            //ROS_DEBUG_STREAM("Twist received");
        }
    }

private:
    // Nodehandle
	ros::NodeHandle n_;
    // Odometry subscriber
    ros::Subscriber subLocalise_;
    // Velocity command subscriber
    ros::Subscriber velSub_;
    // Motor command publishers
    ros::Publisher stbdPub_;
    ros::Publisher portPub_;

    double phi_;

    // Controller gains and parameters
    double lin_kp_;
    double lin_kd_;
    double lin_ki_;
    double prev_lin_error_;
    double acc_lin_error_;
    double max_lin_windup_;

    double ang_kp_;
    double ang_kd_;
    double ang_ki_;
    double prev_ang_error_;
    double acc_ang_error_;
    double max_ang_windup_;

    // Holds the velocity command
    geometry_msgs::Twist twistSet_;

    // Variables to control the heartbeat
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
