#include "explorer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explorer_node");   //simple_navigation_goals node starts
    ros::NodeHandle nh;

    Explorer exp{&nh};
    exp.start();
    exp.home_p();

    ros::shutdown();

    return 0;
}