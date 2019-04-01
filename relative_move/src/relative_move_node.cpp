#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose2D.h"
#include "oryxbot_msgs/SetRelativeMove.h"
#include "boost/thread.hpp"

class RelMove
{
public:
    RelMove():m_nh(ros::NodeHandle()),
        m_nh_private(ros::NodeHandle("~")),
        use_amcl(false)
    {
        m_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        sub_odom = m_nh.subscribe("odom_pose2d", 10, &RelMove::pose2d_callback, this);

//        m_nh_private.param<bool>("use_amcl", use_amcl, false);
        m_nh_private.param<double>("p", m_p, 100);
//        m_nh_private.param<double>("i", m_i, 0);
//        m_nh_private.param<double>("d", m_d, 100);

        m_nh_private.param<double>("err", m_err, 0.05);

        srv_relmove = m_nh.advertiseService("relative_move", &RelMove::relative_move, this);


    }

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_amcl_pose;
    ros::ServiceServer srv_relmove;
    ros::Publisher m_pub;
    bool use_amcl;
    double m_p;
//    double m_i;
//    double m_d;
    double m_err;



    geometry_msgs::Pose2D m_pose2d;
//    geometry_msgs::PoseWithCovarianceStamped m_amcl_pose;

    void pose2d_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        m_pose2d = *msg;
        //ROS_INFO_STREAM("get odom msg");
    }
    bool relative_move(oryxbot_msgs::SetRelativeMove::Request &req,
                       oryxbot_msgs::SetRelativeMove::Response &res)
    {
        //get param
	if (req.mode<0 || req.mode >2){
            res.message = "mode [0,2]";
            res.success = false;
            return true;
        }	
	
        geometry_msgs::Pose2D start_pose = m_pose2d;
        geometry_msgs::Pose2D end_pose = start_pose;
	switch (req.mode) {
	case 0:
		end_pose.x = start_pose.x + req.relative_move;		
        	break;
	case 1:
		end_pose.y = start_pose.y + req.relative_move;
		break;
	case 2: 
		end_pose.theta = start_pose.theta + req.relative_move;
		break;	
	}

        double err = fabs(req.relative_move);

        geometry_msgs::Twist vel;
        while( fabs(err) > fabs(m_err) ){

	    switch (req.mode) {
	    case 0:
		vel.linear.x = m_p * err;
		break;
	    case 1:
		vel.linear.y = m_p * err;
		break;
	    case 2:
		vel.angular.z = m_p * err;
		break;
	    }	
            
            
            m_pub.publish(vel);
            usleep(10*1000);
	    switch (req.mode) {
	    case 0:
		err = end_pose.x - m_pose2d.x;
		break;
	    case 1:
		err = end_pose.y - m_pose2d.y;
		break;
	    case 2:
		err = end_pose.theta - m_pose2d.theta;
		break;
	    }

            ROS_INFO_STREAM("err[" << err << "]");
            ros::spinOnce();

        }
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.angular.z = 0;
        m_pub.publish(vel);
        char str[255]= {0};
        sprintf(str, "current pose2d[%3f, %3f, %3f]", m_pose2d.x, m_pose2d.y, m_pose2d.theta);
        //set message
        res.message = std::string(str);
        res.success = true;
        return true;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "relative_move_node");

    RelMove rm;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
