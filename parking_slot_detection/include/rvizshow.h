#ifndef RVIZSHOW_H
#define RVIZSHOW_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include "pose.h"
#include "associate.h"

class RvizDisplay {
public:
    ros::Publisher path_pub;
    ros::Publisher slots_pub;
    // 存储车辆位姿的路径
    std::vector<geometry_msgs::PoseStamped> pose_path;
    
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_stamped;
    void RvizDisplay_init(ros::NodeHandle& nh);

    void displayParkingSpots(const std::vector<AssociatedPair>& worldassociatedPairs);
    void publishPosePath(const PoseData& new_pose);
};

#endif // RVIZSHOW_H