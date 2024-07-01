#include "pose.h"
#include "drawmap.h"
#include "associate.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_pose");
    ros::NodeHandle nh;

    // VehiclePose vehiclePose; 

    // 订阅GPS和IMU话题
    // ros::Subscriber gps_sub = nh.subscribe("/Inertial/gps/fix", 10, VehiclePose::gpsCallback); 
    // VehiclePose VehiclePose(nh);
    VehiclePose VehiclePose(nh);
    AssociatedParkingInfo AssociatedParkingInfo(nh);
    
    ros::spin();
    return 0;
}