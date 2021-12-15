#include "explorer.h"
#include "follower.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_node");   //simple_navigation_goals node starts
    ros::NodeHandle nh;

    Explorer exp{&nh};
    exp.start();
    exp.home_p();
    
    Follower fl{&nh};
    fl.goal_pos(exp.fid_id);
    if(exp.explorer_done)
    {
        fl.start();
        fl.home_p();
    }
    ros::shutdown();
    return 0;

}