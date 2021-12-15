#ifndef EXPLORER_BOT_H
#define EXPLORER_BOT_H

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

class Explorer
{
public:
    Explorer(ros::NodeHandle* nodehandle);
    void detect_aruca();
    bool explorer_done{false};
    void home_p();
    void start();
    int32_t fid_id;
    
private:
    void to_goal(move_base_msgs::MoveBaseGoal);
    void get_goal();
    void arsubcallback(const fiducial_msgs::FiducialTransformArray::ConstPtr&);
    void detect_marker();
;



    bool detect{false};
    ros::NodeHandle my_nh;
    move_base_msgs::MoveBaseGoal ex_home_pos;
    std::array<XmlRpc::XmlRpcValue,5> ex_goal_pos;
    
    tf2_ros::TransformBroadcaster m_br;
    move_base_msgs::MoveBaseGoal explorer_goal[4];
};


#endif