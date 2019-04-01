#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <oryxbot_msgs/ar_pose.h>
//#include <ar_pose_pub/orientation.h>

oryxbot_msgs::ar_pose pose;
ros::Publisher pub;
//ar_pose_pub::orientation rpy

void pose_callback(const ar_track_alvar_msgs::AlvarMarker marker_msgs)
{
    pose.rpy.resize(3);
    if(marker_msgs.confidence == 1) {
        pose.name = marker_msgs.id;
        pose.position.x = marker_msgs.pose.pose.position.x;
        pose.position.y = marker_msgs.pose.pose.position.y;
        pose.position.z = marker_msgs.pose.pose.position.z;
        pose.confidence = marker_msgs.confidence ;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(marker_msgs.pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(pose.rpy[0], pose.rpy[1], pose.rpy[2]);

    } else {
        pose.name = 0;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.rpy[0] = pose.rpy[1] = pose.rpy[2] = 0;
        pose.confidence = 0;
    }
    pub.publish(pose);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_pose");
    ros::NodeHandle n;
    ros::Subscriber sub  = n.subscribe("ar_pose_single", 10, pose_callback);
    pub = n.advertise<oryxbot_msgs::ar_pose>("ar_pose", 10);
    ros::spin();

}
