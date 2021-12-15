#include "explorer.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Explorer::Explorer(ros::NodeHandle* nodehandle):
    my_nh{*nodehandle} {
        ex_home_pos.target_pose.pose.position.x = -4;
        ex_home_pos.target_pose.pose.position.y = 2.5;
        ex_home_pos.target_pose.pose.orientation.w = 1.0;
        get_goal();
    }

void Explorer::get_goal(){
  //Get aruca marker locations from parameter list
  my_nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_1", ex_goal_pos[0]);
  my_nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_2", ex_goal_pos[1]);
  my_nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_3", ex_goal_pos[2]);
  my_nh.getParam("simple_navigation_goals/aruco_lookup_locations/target_4", ex_goal_pos[3]);
  
for(int i=0; i<=3;i++)
  {
    explorer_goal[i].target_pose.header.frame_id = "map";
    explorer_goal[i].target_pose.header.stamp = ros::Time::now();
    explorer_goal[i].target_pose.pose.orientation.w = 1.0;
    explorer_goal[i].target_pose.pose.position.x = ex_goal_pos[i][0];
    explorer_goal[i].target_pose.pose.position.y = ex_goal_pos[i][1];
    ROS_INFO_STREAM("Explorer: "<<ex_goal_pos[i][0]<<","<<ex_goal_pos[i][1]);
  }

}


void Explorer::start()
{
    for(int i = 0; i<4;i++)
    {
        to_goal(explorer_goal[i]);
        detect_marker();
    }
}
void Explorer::home_p(){
    to_goal(ex_home_pos);
    explorer_done = true;
}

void Explorer::to_goal(move_base_msgs::MoveBaseGoal goal_pos){
    MoveBaseClient explorer_client("/explorer/move_base", true);
    ros::Rate loop_rate(10);
    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    bool explorer_goal_sent = false;
    move_base_msgs::MoveBaseGoal explorer_goal;
    while (ros::ok())
    {
        if (!explorer_goal_sent)
      {
          explorer_client.sendGoal(goal_pos);
          explorer_goal_sent = true;
      }
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
          ROS_INFO("Hooray, follower reached a goal");
          break;
      }
      loop_rate.sleep();
    }
}

void Explorer::arsubcallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msgs)
{
  if (!msgs->transforms.empty()) {//check marker is detected
  // broadcaster object
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";
  transformStamped.transform.translation.x = msgs->transforms[0].transform.translation.x + 0.3;
  transformStamped.transform.translation.y = msgs->transforms[0].transform.translation.y + 0.3;
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

void Explorer::detect_marker(){
    detect = false;  
    ros::Subscriber arsub = my_nh.subscribe("fiducial_transforms",100, Explorer::arsubcallback, this);
    ros::Publisher pubex = my_nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);   //robot rotation when in goal
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        
        geometry_msgs::Twist msg;
        msg.angular.z = 0.1;
        pubex.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        pubex.publish(msg); 

        ros::spinOnce(); 
        loop_rate.sleep();
        if(detect)
        break;
    }
    detect = false; 
}