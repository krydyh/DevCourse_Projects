#include <ros/ros.h>
#include <sensor_msgs/LaserScan>
#include <cmath>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace Maze{

template <typename PREC>
class lidar_process
{
public:
    void Lidar_process();
 
    float getResult() float {return X, Y, ran;};

private:
    void setParams(const YAML::Node& config);  


private:
    float X[505];
    float Y[505];
    float ran;
    //ros variable
    ros::NodeHandle mNodeHandler;
    sensor_msgs::LaserScan mLidarMessage;
};
}