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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int32_t fid_id = 10;

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
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

bool detect()
{
if(fid_id!= 10)
{
  return true;
}
else
{
  return false;
}
}

void arsubcallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msgs){
ROS_INFO("Callback call");
if (!msgs->transforms.empty()) {//check marker is detected
//broadcaster object
static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;
//broadcast the new frame to /tf Topic
transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
transformStamped.child_frame_id = "marker_frame";
transformStamped.transform.translation.x = msgs->transforms[0].transform.translation.x;
/*write the remaining code here*/
fid_id = msgs->transforms[0].fiducial_id;

br.sendTransform(transformStamped);
}
ROS_INFO("Callback call exit");
}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  ros::Subscriber arsub = nh.subscribe("fiducial_transforms",1000, arsubcallback);;
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


  XmlRpc::XmlRpcValue my_goal1;
  XmlRpc::XmlRpcValue my_goal2;
  XmlRpc::XmlRpcValue my_goal3;
  XmlRpc::XmlRpcValue my_goal4;
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_1", my_goal1);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_2", my_goal2);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_3", my_goal3);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_4", my_goal4);
  
  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;
  


  //Build goal for explorer
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  // explorer_goal.target_pose.pose.position.x = 7.710214;//
  // explorer_goal.target_pose.pose.position.y = -1.716889;//
  explorer_goal.target_pose.pose.position.x = my_goal1[0];
  explorer_goal.target_pose.pose.position.y = my_goal1[1];
  explorer_goal.target_pose.pose.orientation.w = 1.0;

  //Build goal for follower
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();
  follower_goal.target_pose.pose.position.x = -0.289296;//
  follower_goal.target_pose.pose.position.y = -1.282680;//
  follower_goal.target_pose.pose.orientation.w = 1.0;



  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
  // explorer_client.waitForResult();


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    int goal_reach = 0;
    if (!explorer_goal_sent)     {
      ROS_INFO("Sending goal for explorer");
      // ROS_INFO("%f\n",my_goal[0]);
      // ROS_INFO("%f\n",my_goal[1]);
      explorer_client.sendGoal(explorer_goal);//this should be sent only once
      explorer_goal_sent = true;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        while(!detect())
        {
          geometry_msgs::Twist msg;
          msg.linear.x = 0;
          msg.linear.y = 0;
          msg.angular.z = 0.1;
          pubex.publish(msg);
        }
      goal_reach ++;
      ROS_INFO("Hooray, robot reached goal");
    }
    // if (!follower_goal_sent) {
    //   ROS_INFO("Sending goal for follower");
    //   follower_client.sendGoal(follower_goal);//this should be sent only once
    //   follower_goal_sent = true;
    // }
    // if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //   ROS_INFO("Hooray, robot reached goal");
    // }
    broadcast();
    listen(tfBuffer);
    ros::spinOnce(); //uncomment this if you have subscribers in your code
    loop_rate.sleep();
  }


}