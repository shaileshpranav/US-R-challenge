#include "follower.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


Follower::Follower(ros::NodeHandle* nodehandle) :
    my_nh{*nodehandle} {
        fl_home_pos.target_pose.pose.position.x = -4;
        fl_home_pos.target_pose.pose.position.y = 3.5;
        fl_home_pos.target_pose.pose.orientation.w = 1.0;

        // goal_pos();
}


void Follower::start(){
    for(int i = 0;i < 4; i++ )
    {
        to_goal(follower_goal[i]);

    }
}
void Follower::home_p()
{
    to_goal(fl_home_pos);
}

void Follower::to_goal(move_base_msgs::MoveBaseGoal goal_pos){
    MoveBaseClient follower_client("/follower/move_base", true);
    while (!follower_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for follower");
    }
    bool follower_goal_sent = false;
    // move_base_msgs::MoveBaseGoal follower_goal;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (!follower_goal_sent)
      {
          follower_client.sendGoal(goal_pos);
         follower_goal_sent = true;
      }
      if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
          ROS_INFO("Hooray, follower reached a goal");
          break;
      }
      loop_rate.sleep();
    }
}

void Follower::fol_goal_posfn(double x,double y, int32_t id)
{
  ROS_INFO("folfn x = %f y = %f",x,y);
  
switch (id)
{
  case 0:
    follower_goal_pos[0][0] = x;
    follower_goal_pos[0][1] = y;
    break;
  case 1:
    follower_goal_pos[1][0] = x;
    follower_goal_pos[1][1] = y;
    break;
  case 2:
    follower_goal_pos[2][0] = x;
    follower_goal_pos[2][1] = y;
    break;
  case 3:
    follower_goal_pos[3][0] = x;
    follower_goal_pos[3][1] = y;
    break;
  default:
    break;
}
}


void Follower::listen(tf2_ros::Buffer& tfBuffer,int32_t id) {
//   if(detect == true && on_goal == true){
  //for listener
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );
    
    fol_goal_posfn(trans_x,trans_y,id);
    ROS_INFO("x = %f y = %f z = %f",trans_x,trans_y,trans_z);  
  }
  catch (tf2::TransformException& ex) {
    // ROS_WARN("%s", ex.what());
    ros::Duration(2.0).sleep();
  }
// }
}

void Follower::goal_pos(int32_t id)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        listen(tfBuffer,id);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

