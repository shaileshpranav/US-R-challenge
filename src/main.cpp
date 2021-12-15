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
bool detect{false};
std::array<std::array<double,2>,4> follower_goal_pos;
bool on_goal{false};

void fol_goal_posfn(double x,double y)
{
  ROS_INFO("folfn x = %f y = %f",x,y);
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

void arsubcallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msgs)
{
  if (!msgs->transforms.empty()) {//check marker is detected
  // broadcaster object
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";
  transformStamped.transform.translation.x = msgs->transforms[0].transform.translation.x;
  transformStamped.transform.translation.y = msgs->transforms[0].transform.translation.y;
  transformStamped.transform.translation.z = msgs->transforms[0].transform.translation.z;
  transformStamped.transform.rotation.x = msgs->transforms[0].transform.rotation.x;
  transformStamped.transform.rotation.y = msgs->transforms[0].transform.rotation.y;
  transformStamped.transform.rotation.z = msgs->transforms[0].transform.rotation.z;
  transformStamped.transform.rotation.w = msgs->transforms[0].transform.rotation.w;
  /*write the remaining code here*/
  fid_id = msgs->transforms[0].fiducial_id;
  detect = true;
  br.sendTransform(transformStamped);
  }
  else
  {
    detect = false; 

  }
}

void listen(tf2_ros::Buffer& tfBuffer) {
  if(detect == true && on_goal == true){
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
    
    fol_goal_posfn(trans_x,trans_y);
    ROS_INFO("x = %f y = %f z = %f",trans_x,trans_y,trans_z);  
  }
  catch (tf2::TransformException& ex) {
    // ROS_WARN("%s", ex.what());
    ros::Duration(2.0).sleep();
  }
}
}

// void follower_start()
// {
//     for(int i = 0; i<=3;i++)
//   {
//     follower_goal[i].target_pose.header.frame_id = "map";
//     follower_goal[i].target_pose.header.stamp = ros::Time::now();
//     follower_goal[i].target_pose.pose.position.x = follower_goal_pos[i][0];
//     follower_goal[i].target_pose.pose.position.y = follower_goal_pos[i][1];
//     follower_goal[i].target_pose.pose.orientation.w = 1.0;
    
//   }
//     follower_goal[4].target_pose.header.frame_id = "map";
//     follower_goal[4].target_pose.header.stamp = ros::Time::now();
//     follower_goal[4].target_pose.pose.position.x = -4;
//     follower_goal[4].target_pose.pose.position.y = 3.5;
//     follower_goal[4].target_pose.pose.orientation.w = 1.0;
// }

void disp()
{
  for(int t = 0;t<4;t++)
  {
  ROS_INFO("%d follower_goal_pos x = %f y = %f", t, follower_goal_pos[t][0],follower_goal_pos[t][1]);
  }
}

void tolerance()
{
  for(int i = 0; i<4;i++)
  {
  if(follower_goal_pos[i][0]>=8)
    follower_goal_pos[i][0] = 7.8;
  if(follower_goal_pos[i][0]<-5)
    follower_goal_pos[i][0] = -4.8;
  if(follower_goal_pos[i][1]>3.8)
    follower_goal_pos[i][1] = 3.2;
  if(follower_goal_pos[i][1]<-3.8)
    follower_goal_pos[i][1] = -3.3;
  }
}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");   //simple_navigation_goals node starts
  ros::NodeHandle nh;

  ros::Subscriber arsub = nh.subscribe("fiducial_transforms",100, arsubcallback);   //aruco marker detect
  ros::Publisher pubex = nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);   //robot rotation when in goal

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


  //Get aruca marker locations from parameter list
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_1", ex_goal_pos[0]);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_2", ex_goal_pos[1]);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_3", ex_goal_pos[2]);
  nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_4", ex_goal_pos[3]);
  
  move_base_msgs::MoveBaseGoal explorer_goal[5];
  move_base_msgs::MoveBaseGoal follower_goal[5];

  //goal positions for explorer
  for(int i=0; i<=3;i++)
  {
    explorer_goal[i].target_pose.header.frame_id = "map";
    explorer_goal[i].target_pose.header.stamp = ros::Time::now();
    explorer_goal[i].target_pose.pose.orientation.w = 1.0;
    explorer_goal[i].target_pose.pose.position.x = ex_goal_pos[i][0];
    explorer_goal[i].target_pose.pose.position.y = ex_goal_pos[i][1];
  }
  //home position for explorer
    explorer_goal[4].target_pose.header.frame_id = "map";
    explorer_goal[4].target_pose.header.stamp = ros::Time::now();
    explorer_goal[4].target_pose.pose.orientation.w = 1.0;
    explorer_goal[4].target_pose.pose.position.x = -4;
    explorer_goal[4].target_pose.pose.position.y = 2.5;
  
  //goal positions for follower
    for(int i = 0; i<=3;i++)
  {
    follower_goal[i].target_pose.header.frame_id = "map";
    follower_goal[i].target_pose.header.stamp = ros::Time::now();
    follower_goal[i].target_pose.pose.position.x = follower_goal_pos[i][0]; 
    follower_goal[i].target_pose.pose.position.y = follower_goal_pos[i][1]; 
    follower_goal[i].target_pose.pose.orientation.w = 1.0;
    
  }
  //home position for follower
    follower_goal[4].target_pose.header.frame_id = "map";
    follower_goal[4].target_pose.header.stamp = ros::Time::now();
    follower_goal[4].target_pose.pose.position.x = -4;
    follower_goal[4].target_pose.pose.position.y = 3.5;
    follower_goal[4].target_pose.pose.orientation.w = 1.0;


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
  int cnt_ex = 0;   //Number of goals visited by explorer
  int cnt_fl = 0;
  bool test{true};

  while (ros::ok()) {
    if(cnt_ex<5)
    {
      if (!explorer_goal_sent)
      {
        std::cout<<"Sending goal";
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal[cnt_ex]);      //goal sent to explorer
        explorer_goal_sent = true;
      }
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
        on_goal = true;
        ROS_INFO("CNT_EX = %d",cnt_ex);
        if(cnt_ex!=4)
        {
          do
          {
            geometry_msgs::Twist msg;
            msg.angular.z = 0.1;
            pubex.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
          } while(!detect);
          ROS_INFO("CALLING LISTENER");
          listen(tfBuffer);   //transform frame from camera frame to map
        }
        detect = false;
        ros::spinOnce();
        loop_rate.sleep();

        cnt_ex++;
        explorer_goal_sent = false;
        ROS_INFO("Hooray, Explorer reached goal");
        on_goal = false;
      }
    }

    if(cnt_ex>4)
    {
      // tolerance();
    if (!follower_goal_sent) {
      disp();
      ROS_INFO("Sending goal for follower");
      ROS_INFO("cnt_fl = %d follower_goal_pos x = %f y = %f",cnt_fl, follower_goal_pos[cnt_fl][0],follower_goal_pos[cnt_fl][1]);
      follower_client.sendGoal(follower_goal[cnt_fl]);    //Sending goal position to follower
      follower_goal_sent = true;
    }
    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      
      follower_goal_sent = false;
      cnt_fl++;
      ROS_INFO("Hooray, follower reached a goal");
      }
    }
    if(cnt_fl==5)
    {
      ROS_INFO("Completed...Shutting down");
      ros::shutdown();
    }
    loop_rate.sleep();
  }


}