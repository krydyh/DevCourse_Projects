#ifndef CUSTOM_FOLLOW_WAYPOINTS_HPP
#define CUSTOM_FOLLOW_WAYPOINTS_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <thread>
#include <atomic>

class State {
public:
    virtual std::string execute() = 0;
};

class GetPath : public State {
public:
    GetPath();
    std::string execute() override;
    const std::vector<geometry_msgs::PoseWithCovarianceStamped>& getWaypoints() const;

private:
    void pathResetCallback(const std_msgs::Empty::ConstPtr& msg);
    void pathReadyCallback(const std_msgs::Empty::ConstPtr& msg);
    void waypointsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    std::atomic<bool> path_reset_;
    std::atomic<bool> path_ready_;
    std::atomic<bool> received_waypoint_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> waypoints_;
    ros::Publisher pose_array_pub_;
    ros::Subscriber path_reset_sub_;
    ros::Subscriber path_ready_sub_;
    ros::Subscriber waypoints_sub_;
    geometry_msgs::PoseArray poseArray_;
};

class FollowPath : public State {
public:
    FollowPath(const std::vector<geometry_msgs::PoseWithCovarianceStamped>& waypoints);
    std::string execute() override;

private:
    const std::vector<geometry_msgs::PoseWithCovarianceStamped>& waypoints_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
};

class PathComplete : public State {
public:
    PathComplete();
    std::string execute() override;
};

#endif // CUSTOM_FOLLOW_WAYPOINTS_HPP
