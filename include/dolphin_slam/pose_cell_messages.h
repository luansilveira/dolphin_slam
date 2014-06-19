#ifndef POSE_CELL_MESSAGES_H
#define POSE_CELL_MESSAGES_H
#include <dolphin_slam/ExperienceEvent.h>
#include <visualization_msgs/Marker.h>

namespace dolphin_slam
{

class PoseCellNetwork;

class PoseCellMessages
{
public:
    void createExperienceEventMessage(PoseCellNetwork *pc, dolphin_slam::ExperienceEvent &message);
    void createNetworkActivityMessageXY(PoseCellNetwork *pc,visualization_msgs::Marker &message);
    void createNetworkActivityMessageXYYaw(PoseCellNetwork *pc,visualization_msgs::Marker &message);
    void createNetworkActivityMessageYaw(PoseCellNetwork *pc,visualization_msgs::Marker &message);
    void createNetworkActivityMessageXYZ(PoseCellNetwork *pc,visualization_msgs::Marker &message);


};


} // namespace dolphin_slam


#endif // POSE_CELL_MESSAGES_H
