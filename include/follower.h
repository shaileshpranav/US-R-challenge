#ifndef FOLLOWER_BOT_H
#define FOLLOWER_BOT_H

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <std_msgs/String.h>

class Follower
{
public:
    Follower(ros::NodeHandle* nodehandle);
    void goal_pos(int32_t);
    void home_p();
    void start();

private:
    void to_goal(move_base_msgs::MoveBaseGoal);
    void listen(tf2_ros::Buffer&, int32_t);
    void get_fl_goal();
    void fol_goal_posfn(double, double, int32_t);
    

    ros::NodeHandle my_nh;
    move_base_msgs::MoveBaseGoal fl_home_pos;
    std::array<XmlRpc::XmlRpcValue,4> fl_goal_pos;
    move_base_msgs::MoveBaseGoal follower_goal[4];

};


#endif