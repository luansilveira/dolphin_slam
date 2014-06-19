#include<dolphin_slam/pose_cell_network.h>
#include<ros/ros.h>
#include<boost/thread.hpp>

int main(int argc, char **argv){


    ros::init(argc, argv, "pose_cell_network");


    dolphin_slam::PoseCellNetwork poseCellNetwork;

    poseCellNetwork.loadConfig();

    poseCellNetwork.connectROSTopics();

    poseCellNetwork.connectROSServices();

//    poseCellNetwork.createROSTimers();

    ros::spin();

    ROS_INFO_STREAM("Shutting Down Pose Cell Network");

}
