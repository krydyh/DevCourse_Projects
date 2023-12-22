// input : lidar message
// output : x, y  of the detected obstacles
// written by team2
// 2023. 06. 05

//
#include "maze_runner/lidar_process.hpp"

namespace Maze{

template <typename PREC>
void lidar_process<PREC>::setParams(const YAML::Node& config)
{
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    LD = config["TOPIC"]["LEFT_DOWN"].as<float, float>();
    RU = config["TOPIC"]["RIGHT_UP"].as<float, float>();
    

}

template <typename PREC>
void lidar_process<PREC> :: Lidar_process()
{

    // lidar node subscribe
    mSubscriber = mNodeHandler.subscribe(mtopic, mLaserScan, &lidar_process::callback, queue_size = mQueueSize);

}


// calculation

template <typename PREC>
void lidar_process<PREC>::callback(const sensor_msgs::LaserScan& message)
{
    float ran = message.ranges;
    float lidar_increment = message.angle_increment

    if ran.size() != 505{
        return 0;
    }

    for (int i = 0; i< 505; i++)
    {
        float rad = i * lidar_increment;
        float x = ran[i] * cosine(rad);
        float y = -ran[i] * sine(rad);
        int pos_X_min = 128;
        int neg_X_max = -128

        X[i] = x;
        Y[i] = y;
        
    }


}
}

