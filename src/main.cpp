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
#include <iostream>
#include <array>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



int32_t fid_id;
bool detect;
int goal_reach = 0;

void broadcast() {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}
std::array<std::array<int,2>,4> follower_goal_pos;

void fol_goal_posfn(int x,int y)
{
switch (fid_id)
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
void listen(tf2_ros::Buffer& tfBuffer) {
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
    // if(detect == true){
    fol_goal_posfn(trans_x,trans_y);
    // ROS_INFO("x = %d y = %d z = %d",trans_x,trans_y,trans_z);
    // }
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}







void arsubcallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msgs){
if (!msgs->transforms.empty()) {//check marker is detected
// broadcaster object
static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;
//broadcast the new frame to /tf Topic
transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
transformStamped.child_frame_id = "marker_frame";
transformStamped.transform.translation.x = msgs->transforms[0].transform.translation.x;
/*write the remaining code here*/
fid_id = msgs->transforms[0].fiducial_id;
detect = true;
ROS_INFO("fid_id = %d",fid_id);
br.sendTransform(transformStamped);
}
else
{
  detect = false;
  // ROS_INFO("Detect false");
}
// ROS_INFO("Callback call exit");
}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  ros::Subscriber arsub = nh.subscribe("fiducial_transforms",100, arsubcallback);
  ros::Publisher pubex=nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);
  ros::Publisher pubfl=nh.advertise<geometry_msgs::Twist>("follower/cmd_vel", 100);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  std::array<XmlRpc::XmlRpcValue,5> ex_goal_pos;

  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_1", ex_goal_pos[0]);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_2", ex_goal_pos[1]);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_3", ex_goal_pos[2]);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_4", ex_goal_pos[3]);
  
  move_base_msgs::MoveBaseGoal explorer_goal[5];
  move_base_msgs::MoveBaseGoal follower_goal[5];

  for(int i=0; i<=3;i++)
  {
    explorer_goal[i].target_pose.header.frame_id = "map";
    explorer_goal[i].target_pose.header.stamp = ros::Time::now();
    explorer_goal[i].target_pose.pose.orientation.w = 1.0;
    explorer_goal[i].target_pose.pose.position.x = ex_goal_pos[i][0];
    explorer_goal[i].target_pose.pose.position.y = ex_goal_pos[i][1];
  }
    explorer_goal[4].target_pose.header.frame_id = "map";
    explorer_goal[4].target_pose.header.stamp = ros::Time::now();
    explorer_goal[4].target_pose.pose.orientation.w = 1.0;
    explorer_goal[4].target_pose.pose.position.x = -4;
    explorer_goal[4].target_pose.pose.position.y = 2.5;

  for(int i = 0; i<=3;i++)
  {
    follower_goal[i].target_pose.header.frame_id = "map";
    follower_goal[i].target_pose.header.stamp = ros::Time::now();
    follower_goal[i].target_pose.pose.position.x = follower_goal_pos[i][0];//
    follower_goal[i].target_pose.pose.position.y = follower_goal_pos[i][1];//
    follower_goal[i].target_pose.pose.orientation.w = 1.0;
    
  }
    follower_goal[4].target_pose.header.frame_id = "map";
    follower_goal[4].target_pose.header.stamp = ros::Time::now();
    follower_goal[4].target_pose.pose.position.x = -4;//
    follower_goal[4].target_pose.pose.position.y = 3.5;//
    follower_goal[4].target_pose.pose.orientation.w = 1.0;

  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
  // explorer_client.waitForResult();


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
  int cnt_ex = 0;
  int cnt_fl = 0;
  int test_i = 0;
  while (ros::ok()) {
    
    if (!explorer_goal_sent && cnt_ex<5)
    {
      std::cout<<"Sending goal";
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goal[cnt_ex]);//this should be sent only once
      explorer_goal_sent = true;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      if(cnt_ex<5)
      {
        ROS_INFO("cnt_ex not yet 5");
        while(!detect)
        {
          geometry_msgs::Twist msg;
          msg.angular.z = 0.1;
          pubex.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
      cnt_ex++;
      explorer_goal_sent = false;
      ROS_INFO("Hooray, Explorer reached goal");
    }
    if(cnt_ex >= 5)
    {
      ROS_INFO("cnt_ex = 5");
      if(test_i<4)
      {
      ROS_INFO("x = %d y = %d",follower_goal_pos[test_i][0],follower_goal_pos[test_i][1]);
      test_i++;
      }
    //       geometry_msgs::Twist msg;
    //       msg.linear.x = 0;
    //       msg.angular.z = 0;
    //       pubex.publish(msg);
    }
    if(!follower_goal_pos[3].empty())
    {
    if (!follower_goal_sent && cnt_fl<5) {
      ROS_INFO("Sending goal for follower");
      follower_client.sendGoal(follower_goal[cnt_fl]);//this should be sent only once
      follower_goal_sent = true;
    }
    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      if(cnt_fl<6)
      {
      follower_goal_sent = false;
      cnt_fl++;
      ROS_INFO("Hooray, robot reached goal");
      }
      else{
        ros::shutdown();
      }
    }
    }
    broadcast();
    listen(tfBuffer);
    ros::spinOnce(); //uncomment this if you have subscribers in your code
    loop_rate.sleep();
  }


}