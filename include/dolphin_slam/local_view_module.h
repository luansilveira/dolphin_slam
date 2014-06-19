#ifndef LOCAL_VIEW_MODULE_H
#define LOCAL_VIEW_MODULE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dolphin_slam/LocalViewNetwork.h>
#include <dolphin_slam/LocalViewCell.h>
#include <dolphin_slam/ExecutionTime.h>


#include <string>

#include <bag_of_words.h>


#include <boost/foreach.hpp>

#include <fstream>
#include "time_monitor.h"

#define foreach BOOST_FOREACH

namespace dolphin_slam
{

struct Cell
{
    int id_;
    float rate_;
    cv::Mat data_;
};



class LocalViewModule
{
public:
    LocalViewModule();
    ~LocalViewModule();

    void loadConfig();
    void connectROSTopics();

    void createROSTimers();

private:
    //! ROS related functions
    void imageCallback(const sensor_msgs::ImageConstPtr &image);
    int createViewTemplate(const cv::Mat &histogram);
    void publishViewTemplate();
    void publishExecutionTime();

    void timerCallback(const ros::TimerEvent &event);

    bool computeLocalViewCellActivation(const cv::Mat & histogram);

    bool detectChanges();
    bool detectChanges(const cv::Mat &image);

    BoW bag_of_words_;

    std::vector<Cell> cells_;
    int most_active_cell_;
    std::vector <int> active_cells_;

    ros::NodeHandle node_handle_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    ros::Publisher view_template_publisher_;
    ros::Publisher execution_time_publisher_;

    std::string image_topic_;

    std::string image_transport_;

    cv::Mat current_appearance_histogram_;

    float match_threshold_;
    double match_appearance_threshold_;

    int number_of_frames_to_jump_;

    std::ofstream view_template_file_;
    std::ofstream local_view_file_;

    TimeMonitor time_monitor_;

    ros::Timer timer_;

    //! Metrics
    int number_of_created_local_views;
    int number_of_recognized_local_views;
    double execution_time;

    bool has_new_local_view_cell;
    int frames_to_jump_bow_trainer_;



};
} //namespace

#endif // LOCAL_VIEW_MODULE_H
