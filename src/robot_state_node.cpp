#include<dolphin_slam/pose_cell_network.h>
#include<ros/ros.h>

int main(int argc, char **argv){


    ros::init(argc, argv, "robot_state_node");

    dolphin_slam::RobotState robotState;

    robotState.loadConfig();
    robotState.connectROSTopics();
    robotState.createROSServices();

//    robotState.createROSTimers();

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    ROS_INFO_STREAM("Shutting Down Robot State Node");

}
