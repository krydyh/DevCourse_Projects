#include <ros/ros.h>
#include <xycar_msgs/xycar_motor.h>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "maze_runner/lidar_process.hpp"

namespace Maze{
template <typename PREC>
class maze_drive{

public:

    using LidarResultPtr = typename lidar_process<PREC>::Ptr;          ///< Pointer type of MovingAverageFilter

    static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
    static constexpr double kFrameRate = 33.0;               ///< Frame rate
    /**
     * @brief Construct a new maze drive object
     */
    maze_drive();

    /**
     * @brief Run maze drive
     */
    void run();

private:

    void setParams(const YAML::Node& config);

    void speedControl(PREC steeringAngle);
    void speedControl_straight(PREC steeringAngle);

    void drive(PREC steeringAngle);



private:
    // Pointer
    LidarResultPtr mLidarProcess;

    // ROX variables
    ros::NodeHandle mNodeHandler;          ///< Node Hanlder for ROS. In this case Detector and Controler
    ros::Publisher mPublisher;             ///< Publisher to send message about
    std::string mPublishingTopicName;      ///< Topic name to publish
    uint32_t mQueueSize;                   ///< Max queue size for message
    xycar_msgs::xycar_motor mMotorMessage; ///< Message for the motor of xycar

    // Xycar Device variables
    PREC mXycarSpeed;                 ///< Current speed of xycar
    PREC mXycarMaxSpeed;              ///< Max speed of xycar
    PREC mXycarMinSpeed;              ///< Min speed of xycar
    PREC mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
    PREC mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    PREC mDecelerationStep;           ///< How much would deaccelrate xycar depending on threshold
    PREC mDecelerationStep_S;           ///< How much would deaccelrate xycar depending on threshold

    

};




}