#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <queue>

using namespace Eigen;
using namespace std;

class GlobalMap{
public:
    GlobalMap(ros::NodeHandle &n){
        roll = 0.0;
        yaw = 0.0;
        pitch = 0.0;
        //cloud.reset((new pcl::PointCloud<pcl::PointXYZ>));
        tf_sub = n.subscribe("tf", 1000, &GlobalMap::tf_callback, this);
        pointcloud_sub = n.subscribe("scan_matched_points2", 1000, &GlobalMap::pointcloud_callback, this);
        pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("global_pointcloud", 1000);
    }
    void start(){
        while(ros::ok()){
            ros::spinOnce();
        }
    }
private:
    const float keep_prob = 0.2;
    const int queue_size = 50;

    ros::Subscriber tf_sub;
    ros::Subscriber pointcloud_sub;
    ros::Publisher pointcloud_pub;

    geometry_msgs::Transform transform_prev;
    geometry_msgs::Transform transform_curr;
    float roll, yaw, pitch;

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    queue< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_queue;

    int counter = 0;

    void tf_callback(const tf::tfMessage::ConstPtr &tf_msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    bool isTooClose();
};
