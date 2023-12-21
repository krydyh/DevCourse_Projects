// input: x, y of the detected obstacles
// output: speed, steering angle => motor msg
// written by team2
// 2023. 06. 05

#include "maze_runner/maze_drive.hpp"

namespace Maze{
template <typename PREC>
void maze_drive<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mDecelerationStep_S = config["XYCAR"]["DECELERATION_STEP_S"].as<PREC>();

}

template <typename PREC>
void maze_drive<PREC>::run()
{
    while (ros::ok())
    {
        //  variables
        float dist_std = 0.8;
        int Conf_cnt = 0;
        int neg_cnt = 0;
        int deg_point = 60;

        ros::spinOnce();
        PREC lidarX, lidarY, dist = PREC (mLidarProcess->getResult());

        for (int i = 0; i< dist.size(); i++){
            if(!(Conf_cnt > deg_point))
            {
                if (dist[i] >= dist_std) // degree and distance;
                {
                    Conf_cnt ++;
                }
                else
                {
                    neg_cnt++;
                }
                if (neg_cnt > 20)
                {
                    Conf_cnt = 0;
                    neg_cnt = 0;
                }
            }
            path_idx = idx - deg_point/2;
        }

        steeringAngle = x[path_idx] * 0.7; //degree
        speedControl(steeringAngle);
        drive(steeringAngle);
    }
    
}

    
template <typename PREC>
void maze_drive<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}


template <typename PREC>
void maze_drive<PREC>::speedControl_straight(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep_S;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void maze_drive<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    mPublisher.publish(motorMessage);
}





} // namespace
