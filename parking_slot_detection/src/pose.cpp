#include "pose.h"
#include <tf/tf.h>
#include "drawmap.h"
#include "rvizshow.h"

RvizDisplay mydisplay;

bool init=false;
static double EARTH_RADIUS = 6378.137;//地球半径
double roll;
double pitch;
double yaw;
struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
my_pose init_pose;
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
VehiclePose::VehiclePose(ros::NodeHandle& nh) {
    gps_sub= nh.subscribe("/Inertial/gps/fix", 10, &VehiclePose::gpsCallback, this);
    imu_sub = nh.subscribe("/Inertial/imu/data", 10, &VehiclePose::imuCallback, this);
    // path_pub = nh.advertise<nav_msgs::Path>("vehicle_trajectory", 10);

    mydisplay.RvizDisplay_init(nh);
}

void VehiclePose::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 从IMU消息中获取朝向信息（例如，使用四元数转换为欧拉角）
        if(imu_msg->orientation_covariance[0] < 0)
	{
		return;
	}
    tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
                     imu_msg->orientation.z, imu_msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // 更新车辆朝向
    roll = roll*180.0f/M_PI;
	pitch = pitch*180.0f/M_PI;
    yaw = -yaw;
    pose.yaw=yaw;
}

void VehiclePose::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // 从GPS消息中获取位置信息
    if(!init)
    {
        init_pose.latitude = gps_msg->latitude;
        init_pose.longitude = gps_msg->longitude;
        init_pose.altitude = gps_msg->altitude;
        init = true;
    }
    else
    {
    //计算相对位置
        double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
		radLat1 = rad(init_pose.latitude);
        radLong1 = rad(init_pose.longitude);
		radLat2 = rad(gps_msg->latitude);
		radLong2 = rad(gps_msg->longitude);
             //计算x
        delta_long = 0;
	delta_lat = radLat2 - radLat1;  //(radLat1,radLong1)-(radLat2,radLong1)
	if(delta_lat>0)
        x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
        else
	x=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));

        x = x*EARTH_RADIUS*1000;

        //计算y
	delta_lat = 0;
        delta_long = radLong2  - radLong1;   //(radLat1,radLong1)-(radLat1,radLong2)
	if(delta_long>0)
	y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
	else
	y=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
        //double y = 2*asin( sin( delta_lat/2 ) + cos( radLat2 )*cos( radLat2)* sin( delta_long/2 )   );
        y = y*EARTH_RADIUS*1000;

        //计算z
      //  double z = gps_msg_ptr->altitude - init_pose.altitude;
        double z =0;
        pose.x=x;
        pose.y=y;
        pose.z=z;
        pose_path.push_back(pose);
         drawVehicleTrajectory(pose_path);
        // RvizDisplay display;
        mydisplay.publishPosePath(pose);

        // publishPosePath(pose);

    }
    

    // 可以在这里添加处理GPS信息的其他逻辑
}
