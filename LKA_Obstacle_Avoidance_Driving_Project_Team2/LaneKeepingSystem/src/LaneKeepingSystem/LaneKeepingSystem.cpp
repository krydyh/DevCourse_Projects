// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class source file
 * @version 1.1
 * @date 2023-05-02
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = std::make_unique<PIDController<PREC>>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = std::make_unique<MovingAverageFilter<PREC>>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mMovingAverage_for_error= std::make_unique<MovingAverageFilter<PREC>>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE_FOR_ERROR"].as<uint32_t>());
    
    mHoughTransformLaneDetector = std::make_unique<HoughTransformLaneDetector<PREC>>(config);
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mDecelerationStep_S = config["XYCAR"]["DECELERATION_STEP_S"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    ros::Rate rate(kFrameRate);
    while (ros::ok())
    {
        ros::spinOnce();
        if (mFrame.empty())
            continue;

        const auto [leftPosisionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame);

        mMovingAverage->addSample(static_cast<int32_t>((leftPosisionX + rightPositionX) / 2));

        int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());

        int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);

        // moving average window - for reducing oscilation - to consider previous error
        mMovingAverage_for_error->addSample(static_cast<int32_t>(errorFromMid));
        errorFromMid = mMovingAverage_for_error -> getResult();

        // team2
        // As the xycar showed low sensitivity to left steering, we added magic number to the error if the error is positive
        /*
        int Err_std = 40;
        if (errorFromMid >= -Err_std-15 && errorFromMid <=Err_std )
        {
            errorFromMid = 5;
        }
        else
        {
            if (errorFromMid > Err_std)
            {
                errorFromMid += 32;
            }
            // As the xycar showed low sensitivity to right steering, we added magic number to the error if the error is negative
            // 
            if (errorFromMid < -Err_std-15)
            {
                // RIGHT TURN
                errorFromMid -= 4 ;
            }
        }
        */

        /*divide pid for straight lane and curved lane
        team2
        */
        PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));

        int Err_std = 40;
        if (errorFromMid >= -Err_std && errorFromMid <=Err_std )
        {
            // straight lane
            PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput_straight(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));
            speedControl_straight(steeringAngle);
        }
        else
        {
            speedControl(steeringAngle);

        }
        //PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));

        drive(steeringAngle);

        if (mDebugging)
        {
            //std::cout << "lpos: " << leftPosisionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << ", error: "<< errorFromMid << ", speed: "<< mXycarSpeed << std::endl;
            // yujin
            //mPublisher.publish(errorFromMid);
            //LaneKeepingSystem::error_data Err;
            //Err.lpos = leftPosisionX;
            //Err.rpos = rightPositionX;
            //Err.error = errorFromMid;
            mHoughTransformLaneDetector->drawRectangles(leftPosisionX, rightPositionX, estimatedPositionX);
            cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            cv::waitKey(1);
        }
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
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
void LaneKeepingSystem<PREC>::speedControl_straight(PREC steeringAngle)
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
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar