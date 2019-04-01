#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>
#include "oryxbot_msgs/SetArPose.h"
#include "oryxbot_msgs/ar_pose.h"
#include <iostream>

#define X_GOAL (0)
#define Z_GOAL (0.3)
#define THETA_GOAL (0)


// velocity control
#define P_THETA (0.5)
#define P_Y (1)
#define P_X (1)

#define I_THETA (0.5)
#define I_Y (1)
#define I_X (1)

#define D_THETA (0.5)
#define D_Y (1)
#define D_X (1)

#define ADJUST_TIMES (10)
#define REPEAT_TIMES (5)
class PID_Controller
{
public:
    PID_Controller(double p, double i, double d):m_p(p), m_i(i), m_d(d)
    {
        m_last_err = 0;
        m_i_err = 0;
        m_d_err = 0;
    }
    ~PID_Controller() {}
    double PID(double err)
    {
        m_i_err += m_last_err;
        m_d_err = err - m_last_err;
        double k = m_p*err + m_i*m_i_err + m_d*m_d_err;
        m_last_err = err;
        return k;
    }
private:
    double m_p;
    double m_i;
    double m_d;
    double m_last_err;
    double m_i_err;
    double m_d_err;
};

class PoseAdj
{
public:
    PoseAdj();
    ~PoseAdj();
    int flag, tmp;

    void callback(const oryxbot_msgs::ar_pose::ConstPtr& pose_msgs);
    bool adjust(oryxbot_msgs::SetArPose::Request &req, oryxbot_msgs::SetArPose::Response &res);
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceServer service;
    ros::Publisher pub;
    oryxbot_msgs::ar_pose curr_pose;
};

PoseAdj::PoseAdj()
{
    sub = n.subscribe("ar_pose", 10, &PoseAdj::callback, this);
    service = n.advertiseService("/pose_adjust/adjust_service", &PoseAdj::adjust, this);
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    curr_pose.rpy.resize(3);
    curr_pose.rpy[0] = curr_pose.rpy[1] = curr_pose.rpy[2] = 0;
}

PoseAdj::~PoseAdj()
{

}

void PoseAdj::callback(const oryxbot_msgs::ar_pose::ConstPtr& pose_msgs)
{
    curr_pose = *pose_msgs;
    //ROS_INFO_STREAM("curr_position[" << curr_pose.position.x
    //                << "," << curr_pose.position.y
    //                << "," << curr_pose.position.z << "]");
    //ROS_INFO_STREAM("curr_rpy[" << curr_pose.rpy[0]
    //                << "," << curr_pose.rpy[1]
    //                << "," << curr_pose.rpy[2] << "]");

}

bool PoseAdj::adjust(oryxbot_msgs::SetArPose::Request &req, oryxbot_msgs::SetArPose::Response &res)
{
    ros::Rate loop(20);
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;

    double x_err  = curr_pose.position.x - X_GOAL;
    double z_goal[ADJUST_TIMES] = {0};
    for(int i=0; i<ADJUST_TIMES; i++) {
        z_goal[i] = curr_pose.position.z-(curr_pose.position.z - Z_GOAL)*(i)/ADJUST_TIMES;
        ROS_INFO_STREAM("z_goal[" << z_goal[i] << "]");
    }

    bool track_flag = curr_pose.confidence;


    if(req.adjust) {
        for(int k=0; k<REPEAT_TIMES; k++) {	//repeat
            for (int i=0; i<ADJUST_TIMES; i++) {
                ROS_INFO_STREAM("adjust times[" << i << "]");
                double z_err = curr_pose.position.z - z_goal[i];
                double theta_err = curr_pose.rpy[1] - THETA_GOAL;

                PID_Controller theta_pid(P_THETA, I_THETA, D_THETA);
                while((fabs(theta_err) > 0.005)  && track_flag ) {
                    twist.angular.z = -theta_pid.PID(theta_err);
                    twist.linear.y = -(curr_pose.position.z+0.5) * twist.angular.z;
                    pub.publish(twist);
                    loop.sleep();
                    theta_err = curr_pose.rpy[1] - THETA_GOAL;
                    ROS_INFO_STREAM("theta_err:" << theta_err);
                    track_flag = curr_pose.confidence;
                }

                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                pub.publish(twist);
                PID_Controller x_pid(P_Y, I_Y, D_Y);
                while((fabs(x_err) > 0.005)  && track_flag ) {
                    twist.linear.y = -x_pid.PID(x_err);
                    pub.publish(twist);
                    loop.sleep();
                    ROS_INFO_STREAM("x_err:" << x_err);
                    x_err  = curr_pose.position.x - X_GOAL;
                    track_flag = curr_pose.confidence;
                }
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                pub.publish(twist);

                PID_Controller z_pid(P_X, I_X, D_X);
                while((fabs(z_err) > 0.005)  && track_flag ) {
                    twist.linear.x = z_pid.PID(z_err);
                    pub.publish(twist);
                    loop.sleep();
                    ROS_INFO_STREAM("z_err:" << z_err);
                    z_err  = curr_pose.position.z - z_goal[i];
                    track_flag = curr_pose.confidence;
                }
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                pub.publish(twist);
                usleep(500*1000);
            }// end repeat
            if(!track_flag) {
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                pub.publish(twist);
                res.message = "failed";
                res.success = false;
                return true;
            }



            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            pub.publish(twist);

        }

        res.message = "success";
        res.success = true;
        return true;
    } else {
        res.message = "failed";
        res.success = false;
        return true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_adjust");

    PoseAdj PoseAdj;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}
