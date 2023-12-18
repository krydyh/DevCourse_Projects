<<<<<<< HEAD
#include "../include/final_project_team4/custom_follow_waypoints.hpp"
=======
#include "final_project/custom_follow_waypoints.hpp"
>>>>>>> 8026686d3c359f1d286a604cdca9ade6a16ed978

GetPath::GetPath()
: nh_(), path_reset_(false), path_ready_(false), received_waypoint_(false),
  pose_array_pub_(nh_.advertise<geometry_msgs::PoseArray>("waypoints", 1)),
  path_reset_sub_(nh_.subscribe("path_reset", 10, &GetPath::pathResetCallback, this)),
  path_ready_sub_(nh_.subscribe("path_ready", 10, &GetPath::pathReadyCallback, this)),
  waypoints_sub_(nh_.subscribe("my_t3_waypoints", 10, &GetPath::waypointsCallback, this)) {
    poseArray_.header.frame_id = "map";
}

void GetPath::pathResetCallback(const std_msgs::Empty::ConstPtr& msg) {
    path_reset_ = true;
}

void GetPath::pathReadyCallback(const std_msgs::Empty::ConstPtr& msg) {
    path_ready_ = true;
}

void GetPath::waypointsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    waypoints_.push_back(*msg);
    received_waypoint_ = true;
}

std::string GetPath::execute() {
    path_reset_ = false;
    path_ready_ = false;
    received_waypoint_ = false;
    waypoints_.clear();
    poseArray_.poses.clear();

    while (!path_ready_ && ros::ok()) {
        if (received_waypoint_) {
            poseArray_.poses.push_back(waypoints_.back().pose.pose);
            pose_array_pub_.publish(poseArray_);
            received_waypoint_ = false;
        }
        ros::spinOnce();
    }
    return "success";
}

const std::vector<geometry_msgs::PoseWithCovarianceStamped>& GetPath::getWaypoints() const {
    return waypoints_;
}

FollowPath::FollowPath(const std::vector<geometry_msgs::PoseWithCovarianceStamped>& waypoints)
: waypoints_(waypoints), move_base_ac_("move_base", true) {
    ROS_INFO("Connecting to move_base...");
    move_base_ac_.waitForServer();
    ROS_INFO("Connected to move_base.");
}

std::string FollowPath::execute() {
    for (const auto& waypoint : waypoints_) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position = waypoint.pose.pose.position;
        goal.target_pose.pose.orientation = waypoint.pose.pose.orientation;

        double x = waypoint.pose.pose.position.x;
        double y = waypoint.pose.pose.position.y;

<<<<<<< HEAD
        move_base_ac_.sendGoal(goal);
        move_base_ac_.waitForResult();

        if (waypoint.pose.pose.position.x == 6.29 && waypoint.pose.pose.position.y == -0.07) {
                ROS_INFO("\n##########################\n"
                         "##### RECEIVING FOOD #####\n"
                         "##########################");
                ros::Duration(5.0).sleep();
            } else if (waypoint.pose.pose.position.x == 7.05 && waypoint.pose.pose.position.y == -4.07) {
                ROS_INFO("\n############################################\n"
                         "###### WAYPOINT '3' DELIVERY COMPLETE ######\n"
                         "############################################");
            } else if (waypoint.pose.pose.position.x == -4.34 && waypoint.pose.pose.position.y == -4.02) {
                ROS_INFO("\n############################################\n"
                         "###### WAYPOINT '4' DELIVERY COMPLETE ######\n"
                         "############################################");
            } else if (waypoint.pose.pose.position.x == -11 && waypoint.pose.pose.position.y == -0.34) {
                ROS_INFO("\n############################################\n"
                         "###### WAYPOINT '5' DELIVERY COMPLETE ######\n"
                         "############################################");
            } else if (waypoint.pose.pose.position.x == -7.0 && waypoint.pose.pose.position.y == -0.00) {
                ROS_INFO("\n######################################\n"
                         "###### DISHES DELIVERY COMPLETE ######\n"
                         "######################################");
            }    
=======
        if (x == 6.29 && y == -0.07) {
            ROS_INFO("############################################");
            ROS_INFO("############## RECEIVING FOOD ##############");
            ROS_INFO("############################################");
        } else if (x == 7.05 && y == -4.07) {
            ROS_INFO("############################################");
            ROS_INFO("###### WAYPOINT '1' DELIVERY COMPLETE ######");
            ROS_INFO("############################################");
        } else if (x == -4.34 && y == -4.02) {
            ROS_INFO("############################################");
            ROS_INFO("###### WAYPOINT '2' DELIVERY COMPLETE ######");
            ROS_INFO("############################################");
        } else if (x == -11.50 && y == -0.198) {
            ROS_INFO("############################################");
            ROS_INFO("###### WAYPOINT '3' DELIVERY COMPLETE ######");
            ROS_INFO("############################################");
        } else if (x == -7.0 && y == -0.0137) {
            ROS_INFO("############################################");
            ROS_INFO("######### DISHES DELIVERY COMPLETE #########");
            ROS_INFO("############################################");
        }

        move_base_ac_.sendGoal(goal);
        move_base_ac_.waitForResult();
>>>>>>> 8026686d3c359f1d286a604cdca9ade6a16ed978
    }
    return "success";
}

PathComplete::PathComplete() {}

std::string PathComplete::execute() {
<<<<<<< HEAD
    ROS_INFO("\n###############################\n##### REACHED FINISH GATE #####\n###############################");
=======
    ROS_INFO("\n###############################\n\
              ##### REACHED FINISH GATE #####\n\
              ###############################");
>>>>>>> 8026686d3c359f1d286a604cdca9ade6a16ed978
    return "success";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_follow_waypoints_cpp");

    GetPath get_path;
    PathComplete path_complete;

    while (ros::ok()) {
        if (get_path.execute() == "success") {
            const std::vector<geometry_msgs::PoseWithCovarianceStamped>& waypoints = get_path.getWaypoints();
            FollowPath follow_path(waypoints);
            if (follow_path.execute() == "success") {
                path_complete.execute();
            }
        }
    }

    return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> 8026686d3c359f1d286a604cdca9ade6a16ed978
