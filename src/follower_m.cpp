#include "follower.h"

int main(int argc, char** argv) {

    ros::NodeHandle nh;
    Follower fl{&nh};
    
    fl.home_p();

    return 0;
}