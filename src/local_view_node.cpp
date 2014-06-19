#include<dolphin_slam/local_view_module.h>
#include<ros/ros.h>

int main(int argc, char **argv){


    ros::init(argc, argv, "local_view_module");

    dolphin_slam::LocalViewModule localViewModule;

    localViewModule.loadConfig();

    localViewModule.connectROSTopics();

    localViewModule.createROSTimers();


    ros::spin();

    ROS_INFO_STREAM("Shutting Down Local View Module");

}
