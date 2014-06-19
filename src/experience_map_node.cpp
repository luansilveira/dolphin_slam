#include<dolphin_slam/experience_map.h>
#include<ros/ros.h>

int main(int argc, char **argv){


    ros::init(argc, argv, "experience_map");


    dolphin_slam::ExperienceMap experience_map;

    experience_map.loadConfig();

    experience_map.connectROSTopics();

    ros::spin();

    ROS_INFO_STREAM("Shutting Down Experience Map");

    experience_map.storeMaps();

}
