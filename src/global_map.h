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
    pcl::PointCloud<pcl::PointXYZ> cloud;
    void tf_callback(const tf::tfMessage::ConstPtr &tf_msg){
        auto transforms= tf_msg->transforms;
        auto data_frame = string("undefined");
        auto data = transforms[1].transform;
        for(auto it = transforms.begin(); it!=transforms.end(); it++){
			cout<<"frame id of tf_msg : "<<(*it).child_frame_id<<endl;
			if((*it).child_frame_id == string("laser")){
				data = (*it).transform;
				data_frame = string("laser");
			}
		}
        if(data_frame != string("laser")){
			return;
		}
        x = data.translation.x;
        y = data.translation.y;
        z = data.translation.z;
        cout<< x << '\t' << y << '\t' << z <<endl;
    }
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg){
        cout<<"height: "<<cloud_msg->height<<endl;
        cout<<"width: "<<cloud_msg->width<<endl;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        //pcl::PCLPointCloud2 pcl_pc2;
        //pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        //pcl::fromPCLPointCloud2(pcl_pc2, cloud);
        //cout<<cloud.width<<endl;
        //cout<<cloud.heitht<<endl;
    }
    void gridmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
        cout<<"get gmmmmmmmmmmm"<<endl;
    }
};
