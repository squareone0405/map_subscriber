#include "global_map.h"
#include <iostream>
#include <string>

using namespace std;

void GlobalMap::tf_callback(const tf::tfMessage::ConstPtr &tf_msg){
    auto transforms = tf_msg->transforms;
    auto transform = transforms[0].transform;
    if(transforms[0].child_frame_id != string("curr_position"))
        return;
    x = transform.translation.x;
    y = transform.translation.y;
    z = transform.translation.z;
    cout<< x << '\t' << y << '\t' << z <<endl;
}

void GlobalMap::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cout<<"init done"<<endl;
    pcl::fromROSMsg (*cloud_msg, *temp_cloud);
    cout<<"parse msg"<<endl;
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = x;
    trans(1, 3) = y;
    trans(2, 3) = z;
    pcl::transformPointCloud (*temp_cloud, *transformed_cloud, trans);
    cout<<"transformed"<<endl;
    *cloud += *transformed_cloud;
    counter++;
    cout<<"num of clouds: "<<counter<<endl;
    cout<<"num of points: "<<cloud->points.size()<<endl;
    cout<<"cloud height: "<<cloud->height<<endl;
    cout<<"cloud width: "<<cloud->width<<endl;
    if((counter%20) == 0){
        pcl::io::savePCDFileASCII ("/home/odroid/Desktop/output" + std::to_string(counter) + ".pcd", *cloud);
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg (*cloud, msg);
        pointcloud_pub.publish(msg);
    }
}
