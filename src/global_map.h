#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

class GlobalMap{
public:
    GlobalMap(ros::NodeHandle &n){
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
    void tf_callback(const tf::tfMessage::ConstPtr &msg){
        cout<<"get tffff"<<endl;
    }
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
        cout<<"get pccccccc"<<endl;
    }
    void gridmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
        cout<<"get gmmmmmmmmmmm"<<endl;
    }
};
