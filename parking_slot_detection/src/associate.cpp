#include "associate.h"
#include"pose.h"
#include"drawmap.h"

int m=0;
cv::Mat canvas1(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
cv::String windowName1 = "slots";
AssociatedParkingInfo::AssociatedParkingInfo(ros::NodeHandle& nh): vehiclepose(nh) {
    
    image_sub = nh.subscribe("/driver/fisheye/avm/compressed", 10, &AssociatedParkingInfo::imageCallback,this);
    client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
}

// void AssociatedParkingInfo::Datafuse(std::vector<int> Assignment)
// {
//     for(int i = 0; i < Assignment.size(); i++) {
//         if(Assignment[i] == -1) {
//              mapassociatedPairs.push_back(preFrame[i]);
//         } else {
//             int match_index = Assignment[i];
            

//             // matched! update

//         }
//     }
// }


// Function to match the same parking spot between two frames
/*void AssociatedParkingInfo::matchParkingSpots(const std::vector<AssociatedPair>& currentFrame)
 {
    double p1_x,p1_y,p2_x,p2_y,center_x,center_y;
    double c1_x,c1_y,c2_x,c2_y,center1_x,center1_y;
    AssociatedPair updatemap;
    updates.clear();
    
    std::vector<std::vector<double>> cost_matrix(preFrame.size(), std::vector<double>(currentFrame.size(), 10000.0));

    if(currentFrame.front().ID==1) 
    {
           preFrame=currentFrame;
           return;
        }
    else{
    for (size_t i = 0; i < currentFrame.size(); ++i) {
        center1_x=(currentFrame[i].spot.x1+currentFrame[i].spot.x2+currentFrame[i].ocrPoint.x1+currentFrame[i].ocrPoint.x2)/2;
        center1_y=(currentFrame[i].spot.y1+currentFrame[i].spot.y2+currentFrame[i].ocrPoint.y1+currentFrame[i].ocrPoint.y2)/2;
        for (size_t j = 0; j < preFrame.size(); ++j) {
                // Check if the spot numbers are the same
                center_x=(preFrame[j].spot.x1+preFrame[j].spot.x2+preFrame[j].ocrPoint.x1+preFrame[j].ocrPoint.x2)/2;
                center_y=(preFrame[j].spot.y1+preFrame[j].spot.y2+preFrame[j].ocrPoint.y1+preFrame[j].ocrPoint.y2)/2;
                double distance = sqrt(pow(center1_x-center_x, 2) + pow((center1_y - center_y), 2));
                    // Check if the positions are close enough
                float distanceThreshold = 0.6; // Define an appropriate threshold

                cost_matrix[j][i] = distance;
            }
        }
    HungarianAlgorithm hungarian;
    std::vector<int> Assignment;
    double cost = hungarian.Solve(cost_matrix, Assignment);
    for(int i = 0; i < Assignment.size(); i++) {
        if(Assignment[i] == -1) {
             mapassociatedPairs.push_back(preFrame[i]);
        } else {
            int match_index = Assignment[i];
            updatemap.spot.x1=(preFrame[i].spot.x1+currentFrame[match_index].spot.x1)/2;
            updatemap.spot.y1=(preFrame[i].spot.y1+currentFrame[match_index].spot.y1)/2;
            updatemap.spot.x2=(preFrame[i].spot.x2+currentFrame[match_index].spot.x2)/2;
            updatemap.spot.y2=(preFrame[i].spot.y2+currentFrame[match_index].spot.y2)/2;
            updatemap.spot.x3=(preFrame[i].spot.x3+currentFrame[match_index].spot.x3)/2;
            updatemap.spot.y3=(preFrame[i].spot.y3+currentFrame[match_index].spot.y3)/2;
            updatemap.spot.x4=(preFrame[i].spot.x4+currentFrame[match_index].spot.x4)/2;
            updatemap.spot.y4=(preFrame[i].spot.y4+currentFrame[match_index].spot.y4)/2;
            updatemap.ocrPoint.x1=(preFrame[i].ocrPoint.x1+currentFrame[match_index].ocrPoint.x1)/2;
            updatemap.ocrPoint.y1=(preFrame[i].ocrPoint.y1+currentFrame[match_index].ocrPoint.y1)/2;
            updatemap.ocrPoint.x2=(preFrame[i].ocrPoint.x2+currentFrame[match_index].ocrPoint.x2)/2;
            updatemap.ocrPoint.y2=(preFrame[i].ocrPoint.y2+currentFrame[match_index].ocrPoint.y2)/2;
            mapassociatedPairs.push_back(updatemap);
            updates.push_back(updatemap);
            // matched! update
            }
        }
        std::vector<int> birthIndices;
    for (int i = 0; i < Assignment.size(); ++i) {
        if (Assignment[i] != -1 && std::find(birthIndices.begin(), birthIndices.end(), i) == birthIndices.end()) {
            birthIndices.push_back(Assignment[i]);
        }
    }
     for (int idx : birthIndices) {
        updates.push_back(currentFrame[ idx]);
            
        }
    preFrame.clear();
    preFrame=updates;
    drawslot(preFrame);
    }
   

    

}*/

void AssociatedParkingInfo::worldlocationspots(const std::vector<AssociatedPair>& pairspots )
{
    worldassociatedPairs.clear();
    AssociatedPair spotpair;
    AssociatedPair pairspot1;
    for (const auto& pairspot : pairspots){
            pairspot1.spot.x1=256-pairspot.spot.x1;
            pairspot1.spot.x2=256-pairspot.spot.x2;
            pairspot1.spot.x3=256-pairspot.spot.x3;
            pairspot1.spot.x4=256-pairspot.spot.x4;
            pairspot1.spot.y1=256-pairspot.spot.y1;
            pairspot1.spot.y2=256-pairspot.spot.y2;
            pairspot1.spot.y3=256-pairspot.spot.y3;
            pairspot1.spot.y4=256-pairspot.spot.y4;
            pairspot1.ocrPoint.x1=256-pairspot.ocrPoint.x1;
            pairspot1.ocrPoint.y1=256-pairspot.ocrPoint.y1;
            pairspot1.ocrPoint.x2=256-pairspot.ocrPoint.x2;
            pairspot1.ocrPoint.y2=256-pairspot.ocrPoint.y2;
        if(pairspot.spot.x1!=0&&!pairspot.ocrPoint.text.empty()){
    spotpair.spot.x1=(pairspot1.spot.x1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.y1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y1=(pairspot1.spot.y1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.x1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.x2=(pairspot1.spot.x2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.y2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y2=(pairspot1.spot.y2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.x2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.x3=(pairspot1.spot.x3)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.y3)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y3=(pairspot1.spot.y3)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.x3)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.x4=(pairspot1.spot.x4)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.y4)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y4=(pairspot1.spot.y4)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.spot.x4)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.x1=(pairspot1.ocrPoint.x1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.ocrPoint.y1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y1=(pairspot1.ocrPoint.y1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.ocrPoint.x1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.x2=(pairspot1.ocrPoint.x2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.ocrPoint.y2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y2=(pairspot1.ocrPoint.y2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot1.ocrPoint.x2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.text=pairspot.ocrPoint.text;
    spotpair.ID=pairspot.ID;
    worldassociatedPairs.push_back(spotpair);
        }
        else if(pairspot.spot.x1==0&&!pairspot.ocrPoint.text.empty()){
        spotpair.spot.x1=spotpair.spot.y1=0;
        spotpair.spot.x2=spotpair.spot.y2=0;
        spotpair.spot.x3=spotpair.spot.y3=0;
        spotpair.spot.x4=spotpair.spot.y4=0;
    spotpair.ocrPoint.x1=(pairspot.ocrPoint.x1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.ocrPoint.y1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y1=(pairspot.ocrPoint.y1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.ocrPoint.x1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.x2=(pairspot.ocrPoint.x2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.ocrPoint.y2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y2=(pairspot.ocrPoint.y2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.ocrPoint.x2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.text=pairspot.ocrPoint.text;
    spotpair.ID=pairspot.ID;
    worldassociatedPairs.push_back(spotpair);

        }
        else if(pairspot.spot.x1!=0&&pairspot.ocrPoint.text=="")
        {
            spotpair.spot.x1=(pairspot.spot.x1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.y1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y1=(pairspot.spot.y1)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.x1)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.x2=(pairspot.spot.x2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.y2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y2=(pairspot.spot.y2)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.x2)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.x3=(pairspot.spot.x3)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.y3)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y3=(pairspot.spot.y3)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.x3)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.x4=(pairspot.spot.x4)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.y4)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y4=(pairspot.spot.y4)/108 * cos(vehiclepose.pose.yaw) -  (pairspot.spot.x4)/108 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.x1=spotpair.ocrPoint.y1=0;
    spotpair.ocrPoint.x2=spotpair.ocrPoint.y2=0;
    spotpair.ocrPoint.text=pairspot.ocrPoint.text;
    worldassociatedPairs.push_back(spotpair);
        }

    }
        // drawslot(worldassociatedPairs);
    //  matchParkingSpots(worldassociatedPairs);

}


void AssociatedParkingInfo::addFrameAssociatedPairs(const std::vector<AssociatedPair>& framePairs)
{
    AllFramesAssociatedPairs.push_back(framePairs);
    
}


void AssociatedParkingInfo::associateSpotsAndNumbers(const parking_slot_detection::gcn_parking & srv)
{
    double association_distance=0.5;
    m=m+1;
    associatedPairs.clear();
    AssociatedPair pair;
    pair.ID=m;
    std::vector<bool> matched_spot_flags(srv.response.point0_x.size(), false); // 标记车位是否已匹配
    std::vector<bool> matched_number_flags(srv.response.ocrpointx1.size(), false); // 标记车位号是否已匹配
     for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
         for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
                double point0_x = (srv.response.point0_x[i]+srv.response.point1_x[i])/2;
                double point0_y = (srv.response.point0_y[i]+srv.response.point1_y[i])/2;
                double x1 = (srv.response.ocrpointx1[j]+srv.response.ocrpointx2[j])/2;
                double y1 = (srv.response.ocrpointy1[j]+srv.response.ocrpointy2[j])/2;
                double distance = sqrt(pow((x1 - point0_x)/108, 2) + pow((y1 - point0_y)/108, 2));
                if (distance < association_distance) {
                     std::cout<<"enter"<<std::endl;
                    // 将关联的车位和车位号打包
                    pair.spot.x1 = srv.response.point0_x[i];
                    pair.spot.y1 = srv.response.point0_y[i];
                    pair.spot.x2 = srv.response.point1_x[i];
                    pair.spot.y2 = srv.response.point1_y[i];
                    pair.spot.x3 = srv.response.point2_x[i];
                    pair.spot.y3 = srv.response.point2_y[i];
                    pair.spot.x4 = srv.response.point3_x[i];
                    pair.spot.y4 = srv.response.point3_y[i];
                    pair.ocrPoint.x1 = srv.response.ocrpointx1[j];
                    pair.ocrPoint.y1 = srv.response.ocrpointy1[j];
                    pair.ocrPoint.x2 = srv.response.ocrpointx2[j];
                    pair.ocrPoint.y2 = srv.response.ocrpointy2[j];
                    pair.ocrPoint.text = srv.response.texts[j];
                    associatedPairs.push_back(pair); // 存储关联对
                    matched_spot_flags[i] = true;
                    matched_number_flags[j] = true;
                    std::cout<<pair.spot.x1<<","<<pair.spot.y1<<","<<pair.spot.x2<<","<<pair.spot.y2<<std::endl;
                    std::cout<<pair.ocrPoint.text<<std::endl;
                    break;
                    

                }

         }
     }
         // 添加未匹配的车位信息
    for (size_t i = 0; i < matched_spot_flags.size(); ++i) {
        if (!matched_spot_flags[i]) {
            // 填充车位信息，车位号信息留空或设置默认值
            // std::cout<<pair.ocrPoint.x1;
            pair.ocrPoint.text="";
            pair.ocrPoint.x1=pair.ocrPoint.y1=pair.ocrPoint.x2=pair.ocrPoint.y2=0;
            pair.spot.x1 = srv.response.point0_x[i];
            pair.spot.y1 = srv.response.point0_y[i];
            pair.spot.x2 = srv.response.point1_x[i];
            pair.spot.y2 = srv.response.point1_y[i];
            pair.spot.x3 = srv.response.point2_x[i];
            pair.spot.y3 = srv.response.point2_y[i];
            pair.spot.x4 = srv.response.point3_x[i];
            pair.spot.y4 = srv.response.point3_y[i];
            associatedPairs.push_back(pair);
        }
    }
    // 添加未匹配的车位号信息
    for (size_t j = 0; j < matched_number_flags.size(); ++j) {
        if (!matched_number_flags[j]) {
            // 车位信息留空或设置默认值，填充车位号信息
            // pair.spot 留空 ...
            pair.spot.x1=pair.spot.y1=pair.spot.x2=pair.spot.y2=pair.spot.x3=pair.spot.y3=pair.spot.x4=pair.spot.y4=0;
            pair.ocrPoint.x1 = srv.response.ocrpointx1[j];
            pair.ocrPoint.y1 = srv.response.ocrpointy1[j];
            pair.ocrPoint.x2 = srv.response.ocrpointx2[j];
            pair.ocrPoint.y2 = srv.response.ocrpointy2[j];
            pair.ocrPoint.text = srv.response.texts[j];
            associatedPairs.push_back(pair);
            

        }
    }
    addFrameAssociatedPairs(associatedPairs);
    worldlocationspots(associatedPairs);
}

void AssociatedParkingInfo::drawslotwithnumber(const cv::Mat& image)
{
    cv::Scalar color1(255, 0, 0);
    cv::Scalar color(0, 255, 0);
    cv::Mat output_image = image.clone();
for (const auto& pair : associatedPairs) {
            // 绘制车位多边形
             if(pair.spot.x1!=0&&!pair.ocrPoint.text.empty()){
                cv::circle(output_image, cv::Point(pair.spot.x1, pair.spot.y1), 3, color1, 2);
                cv::circle(output_image, cv::Point(pair.spot.x2, pair.spot.y2), 3, color1, 2);
                cv::line(output_image, cv::Point(pair.spot.x1, pair.spot.y1), cv::Point(pair.spot.x2, pair.spot.y2), cv::Scalar(0, 0, 255), 2);
                cv::line(output_image, cv::Point(pair.spot.x1, pair.spot.y1), cv::Point(pair.spot.x3, pair.spot.y3), cv::Scalar(0, 0, 255), 2);
                cv::line(output_image, cv::Point(pair.spot.x2, pair.spot.y2), cv::Point(pair.spot.x4, pair.spot.y4), cv::Scalar(0, 0, 255), 2);
                cv::Rect ocr_rect(cv::Point(pair.ocrPoint.x1, pair.ocrPoint.y1),
                             cv::Point(pair.ocrPoint.x2, pair.ocrPoint.y2));
                cv::rectangle(output_image, ocr_rect, cv::Scalar(255, 0, 0), 2);
                cv::putText(output_image, pair.ocrPoint.text,
                        cv::Point(pair.ocrPoint.x1, pair.ocrPoint.y1 - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255));
                }
        
    }
    cv::imshow("Associated Parking Spots and Numbers", output_image);
    cv::waitKey(1); // 等待按键继续
    
}


void AssociatedParkingInfo::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    canvas1.setTo(cv::Scalar(255, 255, 255));
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);
        // 确保图像尺寸符合预期
        cv::resize(image, image, cv::Size(512, 512));
        cv::imshow(windowName1, image);
        cv::waitKey(1);

        // 转换为ROS图像消息
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
       
        // 创建ROS服务客户端
        // ros::NodeHandle mh;
        // ros::ServiceClient client = mh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");

        // 创建请求和响应消息
        // parking_slot_detection::gcn_parking srv;
        srv.request.image_data = *image_msg;
        
    // while(ros::ok()){
        // 发送请求
        if (client.call(srv))
        {
            
            associateSpotsAndNumbers(srv);
            drawslotwithnumber(image);
            // 成功接收响应
            cv::Mat image_vis = image.clone();
            
            cv::Scalar color(255, 0, 0);
            for (size_t i = 0; i < srv.response.point0_x.size(); ++i)
            {
                int point0_x = srv.response.point0_x[i];
                int point0_y = srv.response.point0_y[i];
                int point1_x = srv.response.point1_x[i];
                int point1_y = srv.response.point1_y[i];
                int point2_x = srv.response.point2_x[i];
                int point2_y = srv.response.point2_y[i];
                int point3_x = srv.response.point3_x[i];
                int point3_y = srv.response.point3_y[i];
                int type = srv.response.types[i];
                cv::circle(image_vis, cv::Point(point0_x, point0_y), 3, color, 2);
                cv::circle(image_vis, cv::Point(point1_x, point1_y), 3, color, 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point1_x, point1_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point2_x, point2_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point1_x, point1_y), cv::Point(point3_x, point3_y), cv::Scalar(0, 0, 255), 2);
            }
            for (size_t i = 0; i < srv.response.ocrpointx1.size(); ++i)
            {
                double x1 = srv.response.ocrpointx1[i];
                double x2 = srv.response.ocrpointx2[i];
                double y1 = srv.response.ocrpointy1[i];
                double y2 = srv.response.ocrpointy2[i];
                string text=srv.response.texts[i];
                cv::Scalar rectangleColor(0, 255, 0); // 绿色，BGR颜色空间
                cv::rectangle(image_vis, cv::Point(x1, y1), cv::Point(x2, y2), rectangleColor, 2);
                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double fontScale = 0.5;
                cv::Scalar textColor(0, 255, 0); // 绿色
                int thickness = 2;
                // 绘制文本
                // cv::putText(image_vis, text, cv::Point(x1, y1-10), fontFace, fontScale, textColor, thickness);
            }
            cv::imshow("detected_results", image_vis);
            cv::waitKey(1);

        }
        else {
            ROS_ERROR("Failed to call service");
        }
    }
    // }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }

}


