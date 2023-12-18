#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

double wp_list[6][2] = {
  {6.29, -0.07},
  {7.05, -4.07},
  {-4.34, -4.02},
  {-11, -0.34},
  {0.59, -0.21},
  {-7.0, 0.0}
};

/*double wp_list[5][2] = {
  {6.29, -0.07},
  {7.05, -4.07},
  {-4.34, -4.02},
  {-11, -0.34},
  {-7.0, 0.0}
};*/

void talker() {
    ros::NodeHandle nh;
    ros::Publisher pub_wp = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("my_t3_waypoints", 1);
    ros::Publisher pub_path_ready = nh.advertise<std_msgs::Empty>("path_ready", 1);
    ros::Rate rate(10); // Hz

    geometry_msgs::PoseWithCovarianceStamped my_wp;
    my_wp.header.stamp = ros::Time::now();
    my_wp.header.frame_id = "map";

    double init_roll = 0.0;
    double init_pitch = 0.0;
    double init_yaw = 0.7071;


    ROS_INFO_STREAM("The robot has spawned at waypoint 1\n");

    for (size_t i = 0; i < sizeof(wp_list) / sizeof(wp_list[0]); ++i) {
        my_wp.pose.pose.position.x = wp_list[i][0];
        my_wp.pose.pose.position.y = wp_list[i][1];

        tf::Quaternion quaternion;
        quaternion.setRPY(init_roll, init_pitch, init_yaw);
        tf::quaternionTFToMsg(quaternion, my_wp.pose.pose.orientation);

        while (ros::ok()) {
            if (pub_wp.getNumSubscribers() > 0) {
                pub_wp.publish(my_wp);
                break;
            }
            ROS_INFO("Wait for 'waypoints' topic");
            rate.sleep();
        }

        if(i<4)
            ROS_INFO_STREAM("Published waypoint number " << i+2);
        //else if(i==5)
        //    ROS_INFO_STREAM("Published waypoint number " << i+1);

        ros::Duration(2).sleep();
    }

    std_msgs::Empty start_command;

    while (ros::ok()) {
        if (pub_path_ready.getNumSubscribers() > 0) {
            pub_path_ready.publish(start_command);
            ROS_INFO("Sent waypoint list execution command");
            break;
        }
        ROS_INFO("Waiting for 'path_ready' topic");
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    try {
        talker();
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("An error occurred: " << e.what());
    }
    return 0;
}
