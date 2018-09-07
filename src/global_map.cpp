#include "global_map.h"
#include <iostream>

using namespace std;

void GlobalMap::tf_callback(const tf::tfMessage::ConstPtr &tf_msg){
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

void GlobalMap::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg){
    cout<<"height: "<<cloud_msg->height<<endl;
    cout<<"width: "<<cloud_msg->width<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *temp_cloud);
    Eigen::Matrix4f trans = Eigen::Matrix4f::Zero();
    trans(0, 3) = x;
    trans(1, 3) = y;
    trans(2, 3) = z;
    trans(3, 3) = 1;
    pcl::transformPointCloud (*temp_cloud, *transformed_cloud, trans);
    *cloud += *transformed_cloud;
    counter++;
    cout<<"counter: "<<counter<<endl;
    if(counter == 20){
        pcl::io::savePCDFileASCII ("/home/squareone/output.pcd", *cloud);
    }
}