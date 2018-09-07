#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class GlobalMap{
public:
    GlobalMap(ros::NodeHandle &n){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        tf_sub = n.subscribe("tf", 1000, &GlobalMap::tf_callback, this);
        pointcloud_sub = n.subscribe("scan_matched_points2", 1000, &GlobalMap::pointcloud_callback, this);
        gridmap_sub = n.subscribe("nav_msgs", 1000, &GlobalMap::gridmap_callback, this);
    }
    void start(){
        while(ros::ok()){
            ros::spinOnce();
        }
    }
private:
    ros::Subscriber tf_sub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber gridmap_sub;
    double x, y, z;
    int counter = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    void tf_callback(const tf::tfMessage::ConstPtr &tf_msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    void gridmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
        cout<<"get gridmap"<<endl;
    }
};
