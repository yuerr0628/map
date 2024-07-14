#include "rvizshow.h"
#include <visualization_msgs/MarkerArray.h>



int spotMarker_id = 0;
void RvizDisplay::RvizDisplay_init(ros::NodeHandle& nh){
    // 初始化ROS发布者和订阅者
    path_pub = nh.advertise<nav_msgs::Path>("vehicle_trajectory", 10);
    slots_pub = nh.advertise<visualization_msgs::Marker>("parking_spots", 10);
}



void RvizDisplay::publishPosePath(const PoseData& pose_data) {
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    pose_stamped.header = path_msg.header;
    pose_stamped.pose.position.x = pose_data.x;
    pose_stamped.pose.position.y = pose_data.y;
    pose_stamped.pose.position.z = pose_data.z;
    path_msg.poses.push_back(pose_stamped);
    path_pub.publish(path_msg);
}

void RvizDisplay::displayParkingSpots(const std::vector<AssociatedPair>& worldassociatedPairs) {
    visualization_msgs::MarkerArray markerArray;
    for (const auto& pair : worldassociatedPairs) {
        visualization_msgs::Marker spotMarker;
        spotMarker.header.frame_id = "map"; // 假设车位在"map"坐标系中
        spotMarker.header.stamp = ros::Time::now();
        spotMarker.ns = "parking_spot"+ std::to_string(pair.ID);;
        spotMarker.id = spotMarker_id++;
        spotMarker.type = visualization_msgs::Marker::LINE_STRIP;
        spotMarker.action = visualization_msgs::Marker::ADD;
        // spotMarker.pose.orientation.w = 1.0;

        // 设置车位坐标点
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x1, pair.spot.y1, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x2, pair.spot.y2, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x3, pair.spot.y3, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x4, pair.spot.y4, 0)));
        // spotMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        geometry_msgs::Point point1;
        point1.x = pair.spot.x1;
        point1.y = pair.spot.y1;
        point1.z = 0; // 假设在二维平面上，z 坐标为 0

        geometry_msgs::Point point2;
        point2.x = pair.spot.x2;
        point2.y = pair.spot.y2;
        point2.z = 0;

        geometry_msgs::Point point3;
        point3.x = pair.spot.x3;
        point3.y = pair.spot.y3;
        point3.z = 0;

        geometry_msgs::Point point4;
        point4.x = pair.spot.x4;
        point4.y = pair.spot.y4;
        point4.z = 0;

        // 将点添加到 Marker 的 points 数组中，形成闭合的多边形
        spotMarker.points.push_back(point1);
        spotMarker.points.push_back(point2);
        spotMarker.points.push_back(point3);
        spotMarker.points.push_back(point4);
        spotMarker.points.push_back(point1); // 闭合多边形，返回第一个点

        // 设置颜色和大小
        spotMarker.color.r = 0.0f; // 红色
        spotMarker.color.g = 1.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);

        markerArray.markers.push_back(spotMarker);

    if(!pair.ocrPoint.text.empty()){
        // 为车位号创建Marker
        visualization_msgs::Marker ocrMarker;
        ocrMarker.header.frame_id= "map";
        ocrMarker.ns = "text_boxes";
        ocrMarker.id = spotMarker_id++;
        ocrMarker.type = visualization_msgs::Marker::LINE_LIST;
        ocrMarker.action = visualization_msgs::Marker::ADD;
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        // 为车位号边框设置点坐标
        geometry_msgs::Point point11;
        point11.x = pair.ocrPoint.x1;
        point11.y = pair.ocrPoint.y1;
        point11.z = 0; // 如果是二维情况，z坐标通常设置为0

        geometry_msgs::Point point12;
        point12.x = pair.ocrPoint.x2;
        point12.y = pair.ocrPoint.y1;
        point12.z = 0;

        geometry_msgs::Point point13;
        point13.x = pair.ocrPoint.x2;
        point13.y = pair.ocrPoint.y2;
        point13.z = 0;

        geometry_msgs::Point point14;
        point14.x = pair.ocrPoint.x1;
        point14.y = pair.ocrPoint.y2;
        point14.z = 0;

        // 添加点以闭合多边形（返回第一个点）
        ocrMarker.points.push_back(point11);
        ocrMarker.points.push_back(point12);
        ocrMarker.points.push_back(point13);
        ocrMarker.points.push_back(point14);
        ocrMarker.points.push_back(point11); // 闭合边框
               // 设置颜色和大小
        ocrMarker.color.r = 1.0f;
        ocrMarker.color.g = 0.0f;
        ocrMarker.color.b = 0.0f;
        ocrMarker.color.a = 1.0f;
        ocrMarker.scale.x = 0.1;
        ocrMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(ocrMarker);
      
       
        // 将车位号作为文本显示
        visualization_msgs::Marker textMarker;
        double centerX = (pair.ocrPoint.x1 + pair.ocrPoint.x2) / 2.0;
        double centerY = (pair.ocrPoint.y1 + pair.ocrPoint.y2) / 2.0;
        geometry_msgs::Point pose_center;
        pose_center.x = centerX;
        pose_center.y = centerY;
        textMarker.header.frame_id = "map"; ;
        textMarker.ns = "text";
        textMarker.id = spotMarker_id++;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.pose.position = pose_center;
        ocrMarker.text = pair.ocrPoint.text;
        ocrMarker.scale.z = 1.0; // 文本大小
                // 设置文本颜色和大小
        textMarker.color.r = 1.0f;
        textMarker.color.g = 1.0f;
        textMarker.color.b = 1.0f;
        textMarker.color.a = 1.0f;
        textMarker.scale.z = 1.0; // 文本高度
        ocrMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(textMarker);
    }

        
    }

    slots_pub.publish(markerArray);
        // // 设置文本位置为车位中心点
        
        // pose_center.z = 0.5; // 稍微抬高z值，确保文本显示在地面上方
        // textMarker.pose.position = tf::pointToMsg(pose_center);
        // textMarker.pose.orientation = tf::createQuaternionMsgFromYaw(0); // 确保文本正面朝向相机


}
    


/*void ocrCallback(const showtrajectory::OCRResult::ConstPtr&ocr_msg)
{
    number_msgs.push_back(ocr_msg); 
  const std::vector<geometry_msgs::Point32>& vertices = ocr_msg->polygon.points;
  //time_1=ocr_msg->stamp;
  word_1=ocr_msg->text;
    double confidence_1=ocr_msg->confidence;
    visualization_msgs::Marker marker;
    marker.header.frame_id= "map";
    marker.ns = "text_boxes";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.w =  imu_quaternion.w;
    // marker.pose.orientation.z =  imu_quaternion.z;
    marker.scale.x = 0.1;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < vertices.size(); i++)
    {
        geometry_msgs::Point p1;
    
        // p1.x = vertices[i].x+pose.pose.position.x;
        // p1.y = vertices[i].y+pose.pose.position.y;
        // p1.z = vertices[i].z+pose.pose.position.z;

        // std::cout << yaw << std::endl;
        p1.x = vertices[i].y * cos(yaw) -  vertices[i].x * sin(yaw) +pose.pose.position.x;
        p1.y = vertices[i].y* sin(yaw) +  vertices[i].x * cos(yaw)+pose.pose.position.y;
        p1.z = vertices[i].z+pose.pose.position.z;
        // p1.x = vertices[i].x * cos(yaw) -  vertices[i].y * sin(yaw) +pose.pose.position.x;
        // p1.y = vertices[i].x* sin(yaw) +  vertices[i].y * cos(yaw)+pose.pose.position.y;
        // p1.z = vertices[i].z+pose.pose.position.z;
        
        
        geometry_msgs::Point p2;
        p2.x =  vertices[(i + 1) % vertices.size()].y * cos(yaw) -  vertices[(i + 1) % vertices.size()].x * sin(yaw)+pose.pose.position.x;
        p2.y = vertices[(i + 1) % vertices.size()].y * sin(yaw) +  vertices[(i + 1) % vertices.size()].x * cos(yaw) +pose.pose.position.y;
        p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;
        // p2.x =  vertices[(i + 1) % vertices.size()].x * cos(yaw) -  vertices[(i + 1) % vertices.size()].y * sin(yaw)+pose.pose.position.x;
        // p2.y = vertices[(i + 1) % vertices.size()].x * sin(yaw) +  vertices[(i + 1) % vertices.size()].y * cos(yaw) +pose.pose.position.y;
        // p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    marker_pub.publish(marker);

    // Calculate the center point of the bounding box
    geometry_msgs::Point center;
    for (const geometry_msgs::Point32& vertex : vertices)
    {
        center.x += vertex.y * cos(yaw) -  vertex.x * sin(yaw)+pose.pose.position.x;
        center.y += vertex.y * sin(yaw) +  vertex.x * cos(yaw)+pose.pose.position.y;
        center.z += vertex.z+pose.pose.position.z;
    }
    center.x /= vertices.size();
    center.y /= vertices.size();
    center.z /= vertices.size();

    visualization_msgs::Marker text_marker;
     text_marker.header.frame_id = "map"; ;
    text_marker.ns = "text";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = center;
    // text_marker.pose.orientation.w =  imu_quaternion.w;
    // text_marker.pose.orientation.z =  imu_quaternion.z;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = ros::Duration(0);
    text_marker.text = ocr_msg->text;

    marker_pub.publish(text_marker);
}
// void update_ocrshow()
// {

// }
void slotsCallback(const showtrajectory::Slots::ConstPtr&slots_msg)
{
    slots_msgs.push_back(slots_msg);
  const std::vector<geometry_msgs::Point32>& vertices = slots_msg->polygon.points;
    visualization_msgs::Marker marker;
    marker.header.frame_id= "map";
    marker.ns = "slots_boxes";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < vertices.size(); i++)
    {
        geometry_msgs::Point p1;
        p1.x = vertices[i].y * cos(yaw) -  vertices[i].x * sin(yaw) +pose.pose.position.x-4.515;
        p1.y = vertices[i].y* sin(yaw) +  vertices[i].x * cos(yaw)+pose.pose.position.y;
        p1.z = vertices[i].z+pose.pose.position.z;

        geometry_msgs::Point p2;
        p2.x =  vertices[(i + 1) % vertices.size()].y * cos(yaw) -  vertices[(i + 1) % vertices.size()].x * sin(yaw)+pose.pose.position.x-4.515;
        p2.y = vertices[(i + 1) % vertices.size()].y * sin(yaw) +  vertices[(i + 1) % vertices.size()].x * cos(yaw) +pose.pose.position.y;
        p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;
        // p2.x =  vertices[(i + 1) % vertices.size()].x * cos(yaw) -  vertices[(i + 1) % vertices.size()].y * sin(yaw)+pose.pose.position.x;
        // p2.y = vertices[(i + 1) % vertices.size()].x * sin(yaw) +  vertices[(i + 1) % vertices.size()].y * cos(yaw) +pose.pose.position.y;
        // p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    slots_boxes_pub.publish(marker);
    // drawmapOnCV(ros_path_, slots_msgs, number_msgs);
}

void licenseCallback(const showtrajectory::OCRResult::ConstPtr&ocr_msg)
{
  const std::vector<geometry_msgs::Point32>& vertices = ocr_msg->polygon.points;
  //time_1=ocr_msg->stamp;
  //word_1=ocr_msg->text;
    double confidence_1=ocr_msg->confidence;
    visualization_msgs::Marker marker;
    marker.header.frame_id= "map";
    marker.ns = "license_boxes";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.w =  imu_quaternion.w;
    // marker.pose.orientation.z =  imu_quaternion.z;
    marker.scale.x = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < vertices.size(); i++)
    {
        geometry_msgs::Point p1;
    
        // p1.x = vertices[i].x+pose.pose.position.x;
        // p1.y = vertices[i].y+pose.pose.position.y;
        // p1.z = vertices[i].z+pose.pose.position.z;

        // std::cout << yaw << std::endl;
        p1.x = vertices[i].y * cos(yaw) -  vertices[i].x * sin(yaw) +pose.pose.position.x-4.515;
        p1.y = vertices[i].y* sin(yaw) +  vertices[i].x * cos(yaw)+pose.pose.position.y;
        p1.z = vertices[i].z+pose.pose.position.z;
        // p1.x = vertices[i].x * cos(yaw) -  vertices[i].y * sin(yaw) +pose.pose.position.x;
        // p1.y = vertices[i].x* sin(yaw) +  vertices[i].y * cos(yaw)+pose.pose.position.y;
        // p1.z = vertices[i].z+pose.pose.position.z;
        
        
        geometry_msgs::Point p2;
        p2.x =  vertices[(i + 1) % vertices.size()].y * cos(yaw) -  vertices[(i + 1) % vertices.size()].x * sin(yaw)+pose.pose.position.x-4.515;
        p2.y = vertices[(i + 1) % vertices.size()].y * sin(yaw) +  vertices[(i + 1) % vertices.size()].x * cos(yaw) +pose.pose.position.y;
        p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;
        // p2.x =  vertices[(i + 1) % vertices.size()].x * cos(yaw) -  vertices[(i + 1) % vertices.size()].y * sin(yaw)+pose.pose.position.x;
        // p2.y = vertices[(i + 1) % vertices.size()].x * sin(yaw) +  vertices[(i + 1) % vertices.size()].y * cos(yaw) +pose.pose.position.y;
        // p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    license_marker_pub.publish(marker);

    // Calculate the center point of the bounding box
    geometry_msgs::Point center;
    for (const geometry_msgs::Point32& vertex : vertices)
    {
        center.x += vertex.y * cos(yaw) -  vertex.x * sin(yaw)+pose.pose.position.x-4.515;
        center.y += vertex.y * sin(yaw) +  vertex.x * cos(yaw)+pose.pose.position.y;
        center.z += vertex.z+pose.pose.position.z;
    }
    center.x /= vertices.size();
    center.y /= vertices.size();
    center.z /= vertices.size();

    visualization_msgs::Marker text_marker;
     text_marker.header.frame_id = "map"; ;
    text_marker.ns = "license";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = center;
    // text_marker.pose.orientation.w =  imu_quaternion.w;
    // text_marker.pose.orientation.z =  imu_quaternion.z;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = ros::Duration(0);
    text_marker.text = ocr_msg->text;

    license_marker_pub.publish(text_marker);
}


int main(int argc,char **argv)
{
    init = false;
    ros::init(argc,argv,"gps_to_rviz");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub=nh.subscribe("/Inertial/gps/fix",10, gpsCallback);
   
    ros::Subscriber sub = nh.subscribe<showtrajectory::OCRResult>("ocr_text_with_box", 1000, ocrCallback);
    // ros::Subscriber licence_sub = nh.subscribe<showtrajectory::OCRResult>("license_with_box", 1000, licenseCallback);
    ros::Subscriber slot_sub = nh.subscribe<showtrajectory::Slots>("slots", 1000, slotsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/Inertial/imu/data", 1, imuCallback);

   marker_pub = nh.advertise<visualization_msgs::Marker>("text_boxes", 10);
   license_marker_pub=nh.advertise<visualization_msgs::Marker>("license_boxes", 10);
    state_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);   
   

    ros::spin();
    return 0;//还可以根据需要对图像进行畸变校正。您可以使用提供的畸变中心坐标和映射系数矩阵来进行校正，使用cv2.undistort()函数可以实现畸变校正。
}*/
