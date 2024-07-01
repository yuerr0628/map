#ifndef ASSOCIATE_H
#define ASSOCIATE_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "parking_slot_detection/gcn_parking.h" // 假设这是包含srv定义的头文件
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string> 
#include"pose.h"

#include"../include/Hungarian.h"
using std::string;

struct ParkingSpot {
    double x1, y1, x2, y2, x3, y3, x4, y4;
};

struct OCRPoint {
    std::string text; // 车位号ID
    double x1, y1, x2, y2;
};

struct AssociatedPair {
    ParkingSpot spot; // 关联的车位信息
    OCRPoint ocrPoint; // 关联的车位号信息
    int ID; // 帧id
};


class AssociatedParkingInfo {
public:
    AssociatedParkingInfo(ros::NodeHandle& nh);
    ros::Subscriber image_sub;
    parking_slot_detection::gcn_parking srv;
    ros::ServiceClient client;
    VehiclePose vehiclepose;
    void associateSpotsAndNumbers(const parking_slot_detection::gcn_parking &srv);
    void drawslotwithnumber(const cv::Mat& image);
    void addFrameAssociatedPairs(const std::vector<AssociatedPair>& framePairs);
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
    void matchParkingSpots(const std::vector<AssociatedPair>& currentFrame);
    void worldlocationspots(const std::vector<AssociatedPair>& pairspots );
    void Datafuse(std::vector<int> Assignment);
    std::vector<AssociatedPair> preFrame;
    // 存储所有帧的关联结果
    std::vector<std::vector<AssociatedPair>> AllFramesAssociatedPairs;
    // 存储关联结果的向量
    std::vector<AssociatedPair> associatedPairs;//像素坐标系的车位位置
    std::vector<AssociatedPair> worldassociatedPairs; //世界坐标系下的车位位置
    std::vector<AssociatedPair> updates;
    std::vector<AssociatedPair> mapassociatedPairs; //世界坐标系下的车位位置
};

#endif // ASSOCIATED_PARKING_INFO_H