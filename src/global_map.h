#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <queue>

#include <fstream>

using namespace Eigen;
using namespace std;

class GlobalMap{
public:
    GlobalMap(ros::NodeHandle &n){
        roll = 0.0;
        yaw = 0.0;
        pitch = 0.0;
        tf_sub = n.subscribe("tf", 1000, &GlobalMap::tf_callback, this);
        grid_sub = n.subscribe("map", 1000, &GlobalMap::grid_callback, this);
        //pointcloud_sub = n.subscribe("scan_matched_points2", 1000, &GlobalMap::pointcloud_callback, this);
        //pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("global_pointcloud", 1000);
    }
    void start(){
        while(ros::ok()){
            ros::spinOnce();
        }
    }
private:
    const float keep_prob = 1.0;
    const int queue_size = 10;

    ros::Subscriber tf_sub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber grid_sub;
    ros::Publisher pointcloud_pub;

    geometry_msgs::Transform transform_prev;
    geometry_msgs::Transform transform_curr;
    float roll, yaw, pitch;

    queue< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_queue;

    int counter = 0;

    int map_counter = 0;

    void tf_callback(const tf::tfMessage::ConstPtr &tf_msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    bool isTooClose();
    void sparsification(pcl::PointCloud<pcl::PointXYZ>& dense_cloud);
    void flatten(pcl::PointCloud<pcl::PointXYZ>& cloud_3d);
    void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg){
        map_counter++;
        double resolution = grid_msg->info.resolution;
        int width = grid_msg->info.width;
        int height = grid_msg->info.height;
        ofstream info_out;
        info_out.open("/home/squareone/Desktop/output/info" + to_string(map_counter) + ".txt");
        info_out<<"resolution: "<<resolution<<endl;
        info_out<<"width: "<<width<<endl;
        info_out<<"height: "<<height<<endl;
        info_out<<"origin: "<<endl;
        info_out<<"position: "<<endl;
        info_out<<"x, y, z:"<<grid_msg->info.origin.position.x<<"\t"<<grid_msg->info.origin.position.y<<"\t"<<grid_msg->info.origin.position.z<<endl;
        info_out<<"orientation: "<<endl;
        info_out<<"x, y, z, w:"<<grid_msg->info.origin.orientation.x<<"\t"<<grid_msg->info.origin.orientation.y
            <<"\t"<<grid_msg->info.origin.orientation.z<<"\t"<<grid_msg->info.origin.orientation.w<<endl;
        info_out.close();
        ofstream csv_out;
        csv_out.open("/home/squareone/Desktop/output/" + to_string(map_counter) + ".csv");
        for (int i = 0; i < height; ++i){
            for(int j = 0; j < width; ++j){
                csv_out<<(int)(grid_msg->data[i*width + j]);
                if(j != width - 1)
                    csv_out<<",";
            }
            csv_out<<endl;
        }
        csv_out.close();
        cout<<"get nav_msg---------------------------------------------------"<<map_counter<<endl;
    }
};
