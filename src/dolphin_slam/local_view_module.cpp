#include "local_view_module.h"

const float ROS_TIMER_STEP = 0.25;

namespace dolphin_slam
{

LocalViewModule::LocalViewModule()
{

    number_of_created_local_views = 0;
    number_of_recognized_local_views = 0;
    number_of_frames_to_jump_ = 0;
}

LocalViewModule::~LocalViewModule()
{
    view_template_file_.close();
    local_view_file_.close();
}

bool LocalViewModule::detectChanges(const cv::Mat &image)
{
    cv::Mat appearance_histogram;
    int hist_size = 64;
    int channels = 0;
    double correlation;
    int dim = 1;

    float range[] = { 0, 255 };
    const float *ranges[] = {range};


    //calcula o histograma da imagem
    cv::calcHist(&image,1,&channels,cv::Mat(),appearance_histogram,dim,&hist_size,ranges,true,false);
    cv::normalize(appearance_histogram, appearance_histogram, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());


    if(number_of_created_local_views == 0)
    {
        current_appearance_histogram_ = appearance_histogram;
        return true;
    }
    else
    {
        correlation = compareHist( current_appearance_histogram_, appearance_histogram, CV_COMP_CORREL);


        ROS_DEBUG_STREAM("Difference in appearence = " << correlation);

        //Testa se a correlação é baixa
        if(correlation < match_appearance_threshold_)
        {
            current_appearance_histogram_ = appearance_histogram;
            return true;
        }
        else
        {
            return false;
        }

    }

}


bool LocalViewModule::detectChanges()
{

    static int i = 0;

    ROS_DEBUG_STREAM("frame_number = " << i);

    if(i < number_of_frames_to_jump_)
    {
        i++;
        return false;
    }
    else
    {
        i = 0;
        return true;
    }

}

void LocalViewModule::loadConfig()
{
    std::string dictionary_path;
    std::string dictionary_name;
    int surf_threshold;
    double match_threshold;
    int number_of_groups;
    std::string filename;
    int test_number;

    node_handle_.param<std::string>("image_transport", image_transport_, "compressed");

    node_handle_.param<double>("match_threshold",match_threshold,0.03);
    match_threshold_ = static_cast<float>(match_threshold);

    node_handle_.param<double>("match_appearance_threshold",match_appearance_threshold_,0.03);

    node_handle_.param<int>("number_of_frames_to_jump",number_of_frames_to_jump_,10);

    node_handle_.param<std::string>("bow/dictionary_path",dictionary_path,"config/bow/simulator/");

    node_handle_.param<std::string>("image_topic", image_topic_, "/uwsim/camera1");

    node_handle_.param<int>("frames_to_jump_bow_trainer", frames_to_jump_bow_trainer_, 10);

    node_handle_.param<int>("bow/number_of_groups",number_of_groups,100);
    node_handle_.param<int>("bow/surf_threshold",surf_threshold,100);
    bag_of_words_.setThreshold(surf_threshold);

    if(image_topic_ == "/tazmania/camera/right")
    {
        dictionary_name = dictionary_path + "tazmania_groups_" + boost::lexical_cast<std::string>(number_of_groups)
                + "_threshold_" + boost::lexical_cast<std::string>(surf_threshold) + ".xml";
    }
    else
    {
        dictionary_name = dictionary_path + "features" + boost::lexical_cast<std::string>(surf_threshold)
                + "/" + boost::lexical_cast<std::string>(number_of_groups)+".xml";
    }


    cout << "Dictionary name = " << dictionary_name << endl;

    bag_of_words_.readDictionary(dictionary_name.c_str());

    node_handle_.param<int>("test_number",test_number,1);
    filename = "view_templates" + boost::lexical_cast<std::string>(test_number)+".txt";
    view_template_file_.open(filename.c_str());

    filename = "local_view" + boost::lexical_cast<std::string>(test_number)+".txt";
    local_view_file_.open(filename.c_str());
}

/*! Function to init ROS topics
*   Initialize all topics, creating the subscription and advertises of Local View Module
*
*/
void LocalViewModule::connectROSTopics()
{
    //Subscribers

    //! image_transport declaration
    image_transport::ImageTransport it(node_handle_);

    //! hint to modify the image_transport. Here I use raw transport
    image_transport::TransportHints hints(image_transport_,ros::TransportHints(),node_handle_);

    //! image subscription
    //image_subscriber_ = it.subscribe(image_topic_,1,&LocalViewModule::imageCallback,this,hints);

    //Publishers

    //! \todo Modificar o tipo da mensagem
    view_template_publisher_ = node_handle_.advertise<dolphin_slam::LocalViewNetwork>("local_view_cells",1);

    image_publisher_ = it.advertise("/image_keypoints", 1);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);


}

void LocalViewModule::createROSTimers()
{
    timer_ = node_handle_.createTimer(ros::Duration(0.5), &LocalViewModule::timerCallback,this);

}

void LocalViewModule::timerCallback(const ros::TimerEvent& event)
{
    static int cont = 0;

    has_new_local_view_cell = true;

    most_active_cell_ = cont++;


    //current_view_template_ = cont++;


    publishViewTemplate();
}

/*!
*   \brief Function to publish view template
*/
void LocalViewModule::publishViewTemplate(){

    LocalViewNetwork message;
    Cell *cell;

    message.header.stamp = ros::Time::now();

    message.has_new_cell_ = has_new_local_view_cell;


    message.number_of_cells_ = cells_.size();
    message.most_active_cell_ = most_active_cell_;
    message.active_cells_.resize(active_cells_.size());

    for(int i=0;i<active_cells_.size();i++)
    {
        //! Assert para testar se o id está confizente com o indice do vetor
        cell = &cells_[active_cells_[i]];
        message.active_cells_[i].id_ = cell->id_;
        message.active_cells_[i].rate_ = cell->rate_;


        //! Salva o log no arquivo
        view_template_file_ << cell->id_ << " " << cell->rate_ << " ";
    }
    //! termina a linha no arquivo de log
    view_template_file_ << endl;

    //! Publica a mensagem
    view_template_publisher_.publish(message);

}

/*!
*   \brief Image Callback Function
*
*   Function to receive and process the imagem when arrived at the topic
*   \param image image message
*/
void LocalViewModule::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    cv_bridge::CvImageConstPtr cv_image_ptr;

    cv_bridge::CvImage image_keypoints;

    cv_image_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

//    if(detectChanges(cv_image_ptr->image))
    if(detectChanges())
    {
        time_monitor_.start();

        cv::Mat histogram;
        histogram = bag_of_words_.createHistogram(cv_image_ptr->image);

        try {
            if(computeLocalViewCellActivation(histogram))
            {
                number_of_created_local_views++;
                has_new_local_view_cell = true;
                ROS_DEBUG_STREAM("View Template created. ID = " << cells_.size()-1);
            }else
            {
                has_new_local_view_cell = false;
                ROS_DEBUG_STREAM("View Template recognized.");
            }

        } catch (cv::Exception e) {
            ROS_WARN_STREAM("Histograma inválido");
            return;
        }

        publishViewTemplate();

        time_monitor_.finish();
        time_monitor_.print();

        execution_time = time_monitor_.getDuration();

        publishExecutionTime();

        local_view_file_ << number_of_created_local_views << " " << execution_time << endl;

        //! Publica uma imagem com os keypoints reconhecidos
        cv::Mat image;
        cv::drawKeypoints(cv_image_ptr->image,bag_of_words_.getKeypoints(),image);
        image_keypoints.image = image;
        image_keypoints.encoding = "bgr8";
        image_publisher_.publish(image_keypoints.toImageMsg());

    }

}

void LocalViewModule::publishExecutionTime()
{
    ExecutionTime msg;
    msg.header.stamp = ros::Time::now();

    msg.module = "lv";
    msg.iteration_time = time_monitor_.getDuration();

    execution_time_publisher_.publish(msg);

}


int LocalViewModule::createViewTemplate(const cv::Mat & histogram)
{

    cells_.resize(cells_.size() + 1);
    Cell* cell = &(*(cells_.end() - 1));

    cell->id_ = cells_.size() - 1;

    cell->rate_ = 1.0;
    cell->data_ = histogram.clone();

    return cell->id_;
}



bool LocalViewModule::computeLocalViewCellActivation(const cv::Mat & histogram)
{
    float correlation;

    float major_activation = 0;

    bool ret = false;

    active_cells_.clear();
    foreach (Cell &cell, cells_){

        correlation = cv::compareHist(cell.data_,histogram,CV_COMP_CORREL);

        if(correlation > match_threshold_)
        {
//            cell.rate_ = (correlation - match_threshold_)/(1.0 - match_threshold_);

            cell.rate_ = correlation;

            active_cells_.push_back(cell.id_);
            ret = true;

            if (cell.rate_ > major_activation){
                major_activation = cell.rate_;
                most_active_cell_ = cell.id_;
            }
        }
        else
        {
            cell.rate_ = 0.0;
        }
    }

    //! Não encontrou nenhuma célula parecida
    if(!ret)
    {
        //! Cria uma nova local view
        most_active_cell_ = createViewTemplate(histogram);
        active_cells_.push_back(most_active_cell_);
    }

    return ret;

}

} //namespace
