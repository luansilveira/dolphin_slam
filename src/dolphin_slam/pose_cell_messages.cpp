#include "pose_cell_messages.h"

#include <pose_cell_network.h>

namespace dolphin_slam
{

//! ROS Messages creation

void PoseCellMessages::createExperienceEventMessage(PoseCellNetwork *pc,dolphin_slam::ExperienceEvent &message)
{
    std::vector<int> active_index(4);
    int index[4];
    int i,j,k,l;    //! indices da matriz de neuronios


    //! set view template id
    message.view_template_id = pc->view_template_id_;

    message.rot_delta_position.x = pc->robot_pose_em_.response.rot_delta_x;
    message.rot_delta_position.y = pc->robot_pose_em_.response.rot_delta_y;
    message.rot_delta_position.z = pc->robot_pose_em_.response.delta_z;

    message.delta_yaw = pc->robot_pose_em_.response.delta_o;

    message.ground_truth_position_x = pc->robot_pose_em_.response.ground_truth_position_x;
    message.ground_truth_position_y = pc->robot_pose_em_.response.ground_truth_position_y;
    message.ground_truth_position_z = pc->robot_pose_em_.response.ground_truth_position_z;

    //! set active index
    pc->getActiveNeuron(active_index);

    message.pose_cell_index.resize(4);
    std::copy(active_index.begin(),active_index.end(),message.pose_cell_index.begin());

//    message.activity.number_of_dimensions = pc->number_of_neurons_.size();
//    foreach (int n, pc->number_of_neurons_) {
//        message.activity.dims.push_back(n);
//    }

//    message.activity.data.reserve(pc->neurons_.total());
//    //! para cada neuronio da matriz
//    for(i=0;i<pc->number_of_neurons_[0];i++)
//    {
//        for(j=0;j<pc->number_of_neurons_[1];j++)
//        {
//            for(k=0;k<pc->number_of_neurons_[2];k++)
//            {
//                for(l=0;l<pc->number_of_neurons_[3];l++)
//                {
//                    //! preenche os indices em um vetor
//                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
//                    message.activity.data.push_back(pc->neurons_(index));
//                }
//            }
//        }
//    }

}

void PoseCellMessages::createNetworkActivityMessageXY(PoseCellNetwork *pc,visualization_msgs::Marker & message)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4];
    cv::Mat_<float> activity(pc->number_of_neurons_[0],pc->number_of_neurons_[1]);

    float scale = 0.25;
    Color color;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/network";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::POINTS;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "network";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -scale*pc->number_of_neurons_[0]/2;
    message.pose.position.y = -scale*pc->number_of_neurons_[1]/2;
    message.pose.position.z = 0.0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = scale;
    message.scale.y = scale;
    message.scale.z = scale;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    int number_of_points = pc->number_of_neurons_[0]*pc->number_of_neurons_[1];
    message.points.resize(number_of_points);
    message.colors.resize(number_of_points);

    //! para cada neuronio da matriz
    for(i=0;i<pc->number_of_neurons_[0];i++)
    {
        for(j=0;j<pc->number_of_neurons_[1];j++)
        {
            activity[i][j] = 0;
            for(k=0;k<pc->number_of_neurons_[2];k++)
            {
                for(l=0;l<pc->number_of_neurons_[3];l++)
                {
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
                    activity[i][j] += pc->neurons_(index);
                }
            }
        }
    }

    float max = *std::max_element(activity.begin(),activity.end());
    int point_index = 0;
    //! para cada neuronio da matriz
    //cout << "Activity = " << endl;
    for(i=0;i<pc->number_of_neurons_[0];i++)
    {
        for(j=0;j<pc->number_of_neurons_[1];j++)
        {
            //cout << activity[i][j] << ", ";
            //! set point position
            message.points[point_index].x = scale*i;
            message.points[point_index].y = scale*j;
            message.points[point_index].z = 0;

            color.setColor(activity(i,j)/max);

            //! set point color, according to network activity;
            message.colors[point_index].r = color.getR();
            message.colors[point_index].g = color.getG();
            message.colors[point_index].b = color.getB();
            message.colors[point_index].a = 1.0;

            point_index++;
        }
        //cout << endl;
    }

}

void PoseCellMessages::createNetworkActivityMessageXYYaw(PoseCellNetwork *pc,visualization_msgs::Marker & message)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4];
    int dimensions[3];

    dimensions[0] = pc->number_of_neurons_[0];
    dimensions[1] = pc->number_of_neurons_[1];
    dimensions[2] = pc->number_of_neurons_[3];

    cv::Mat_<float> activity(3,dimensions);

    int act_index[3];

    float scale = 0.25;
    Color color;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/network";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::POINTS;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "network";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -pc->number_of_neurons_[0]/2;
    message.pose.position.y = -pc->number_of_neurons_[1]/2;
    message.pose.position.z = 0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = scale;
    message.scale.y = scale;
    message.scale.z = scale;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    int number_of_points = pc->number_of_neurons_[0]*pc->number_of_neurons_[1]*pc->number_of_neurons_[3];
    message.points.resize(number_of_points);
    message.colors.resize(number_of_points);

    //! para cada neuronio da matriz
    for(i=0;i<pc->number_of_neurons_[0];i++)
    {
        for(j=0;j<pc->number_of_neurons_[1];j++)
        {
            for(l=0;l<pc->number_of_neurons_[3];l++)
            {
                act_index[0] = i; act_index[1] = j; act_index[2] = l;
                activity(act_index) = 0;
                for(k=0;k<pc->number_of_neurons_[2];k++)
                {
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
                    activity(act_index) += pc->neurons_(index);
                }
            }
        }
    }

    float max = *std::max_element(activity.begin(),activity.end());
    int point_index = 0;
    //! para cada neuronio da matriz
    //cout << "Activity = " << endl;
    for(i=0;i<pc->number_of_neurons_[0];i++)
    {
        for(j=0;j<pc->number_of_neurons_[1];j++)
        {
            for(l=0;l<pc->number_of_neurons_[3];l++)
            {
                //! set point position
                message.points[point_index].x = i;
                message.points[point_index].y = j;
                message.points[point_index].z = l;


                act_index[0] = i; act_index[1] = j; act_index[2] = l;
                color.setColor(activity(act_index)/max);

                //! set point color, according to network activity;
                message.colors[point_index].r = color.getR();
                message.colors[point_index].g = color.getG();
                message.colors[point_index].b = color.getB();
                message.colors[point_index].a = 1.0;

                point_index++;
            }
        }
        //cout << endl;
    }

}


void PoseCellMessages::createNetworkActivityMessageXYZ(PoseCellNetwork *pc, visualization_msgs::Marker & message)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4];
    int dimensions[3];

    dimensions[0] = pc->number_of_neurons_[0];
    dimensions[1] = pc->number_of_neurons_[1];
    dimensions[2] = pc->number_of_neurons_[2];

    cv::Mat_<float> activity(3,dimensions);

    int act_index[3];

    float scale = 0.25;
    Color color;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/network";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::POINTS;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "network";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -pc->number_of_neurons_[0]/2;
    message.pose.position.y = -pc->number_of_neurons_[1]/2;
    message.pose.position.z = 0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = scale;
    message.scale.y = scale;
    message.scale.z = scale;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    int number_of_points = pc->number_of_neurons_[0]*pc->number_of_neurons_[1]*pc->number_of_neurons_[2];
    message.points.resize(number_of_points);
    message.colors.resize(number_of_points);

    //! para cada neuronio da matriz
    for(i=0;i<pc->number_of_neurons_[0];i++)
    {
        for(j=0;j<pc->number_of_neurons_[1];j++)
        {
            for(k=0;k<pc->number_of_neurons_[2];k++)
            {
                act_index[0] = i; act_index[1] = j; act_index[2] = k;
                activity(act_index) = 0;
                for(l=0;l<pc->number_of_neurons_[3];l++)
                {
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
                    activity(act_index) += pc->neurons_(index);
                    std::cout << pc->neurons_(index) << " " ;
                }
            }
        }
    }

    float max = *std::max_element(activity.begin(),activity.end());
    int point_index = 0;
    //! para cada neuronio da matriz
    //cout << "Activity = " << endl;
    for(i=0;i<pc->number_of_neurons_[0];i++)
    {
        for(j=0;j<pc->number_of_neurons_[1];j++)
        {
            for(k=0;k<pc->number_of_neurons_[2];k++)
            {
                //! set point position
                message.points[point_index].x = i;
                message.points[point_index].y = j;
                message.points[point_index].z = k;


                act_index[0] = i; act_index[1] = j; act_index[2] = k;
                color.setColor(activity(act_index)/max);

                //! set point color, according to network activity;
                message.colors[point_index].r = color.getR();
                message.colors[point_index].g = color.getG();
                message.colors[point_index].b = color.getB();
                message.colors[point_index].a = 1.0;

                point_index++;
            }
        }
        //cout << endl;
    }

}

void PoseCellMessages::createNetworkActivityMessageYaw(PoseCellNetwork *pc,visualization_msgs::Marker & message)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4];

    std::vector<float> activity(pc->number_of_neurons_[3]);

    float scale = 0.25;
    Color color;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/network";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "network";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -pc->number_of_neurons_[3]/2;
    message.pose.position.y = -5;
    message.pose.position.z = -2;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = scale;
    message.scale.y = scale;
    message.scale.z = scale;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    //! para cada neuronio da matriz
    for(l=0;l<pc->number_of_neurons_[3];l++)
    {
        activity[l] = 0;
        for(i=0;i<pc->number_of_neurons_[0];i++)
        {
            for(j=0;j<pc->number_of_neurons_[1];j++)
            {
                for(k=0;k<pc->number_of_neurons_[2];k++)
                {
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
                    activity[l] += pc->neurons_(index);
                }
            }
        }
    }

    float max = *std::max_element(activity.begin(),activity.end());
    //! para cada neuronio da matriz
    //cout << "Activity = " << endl;
    message.points.resize(activity.size());
    message.colors.resize(activity.size());
    for(l=0;l<activity.size();l++)
    {
        //! set point position
        message.points[l].x = l;
        message.points[l].y = 0;
        message.points[l].z = activity[l];

        color.setColor(activity[i]/max);

        //! set point color, according to network activity;
        message.colors[l].r = color.getR();
        message.colors[l].g = color.getG();
        message.colors[l].b = color.getB();
        message.colors[l].a = 1.0;
    }
}


} //dolphin_slam
