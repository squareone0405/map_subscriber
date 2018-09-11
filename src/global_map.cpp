#include "global_map.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

bool GlobalMap::isTooClose(geometry_msgs::Transform transform_new) {
    float dis = pow(transform.translation.x - transform_new.translation.x, 2) 
    + pow(transform.translation.y - transform_new.translation.y, 2) 
    + pow(transform.translation.z - transform_new.translation.z, 2);
    if(dis < 0.01)
        return false;
    float qx = transform_new.rotation.x;
    float qy = transform_new.rotation.y;
    float qz = transform_new.rotation.z;
    float qw = transform_new.rotation.w;
    float roll_new = std::atan2(2 * (qw * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
    float pitch_new = std::asin(2 * (qw * qy - qx * qz));
    float yaw_new = std::atan2(2 * (qw * qz + qx * qy), (1 - 2 * (qz * qz + qy * qy)));
    float dis_angle = abs(roll_new - roll) + abs(pitch_new - pitch) + abs(yaw_new - yaw);
    if(dis_angle < M_PI / 36)
        return false;
    transform = transform_new;
    roll = roll_new;
    pitch = pitch_new;
    yaw = yaw_new;
    return true;
}

void GlobalMap::tf_callback(const tf::tfMessage::ConstPtr &tf_msg){
    hasMoved = false;
    auto transforms = tf_msg->transforms;
    auto transform_new = transforms[0].transform;
    if(transforms[0].child_frame_id != string("curr_position"))
        return;
    if(isTooClose(transform_new))
        return;  
    hasMoved = true; 
    cout<< transform.translation.x << '\t' << transform.translation.y << '\t' << transform.translation.z <<endl;
}

void GlobalMap::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    if(!hasMoved){
        cout<<"reject! no move!"<<endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *temp_cloud);
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = transform.translation.x;
    trans(1, 3) = transform.translation.y;
    trans(2, 3) = transform.translation.z;
    pcl::transformPointCloud (*temp_cloud, *transformed_cloud, trans);
    *cloud += *transformed_cloud;
    counter++;
    cout<<"num of clouds: "<<counter<<endl;
    cout<<"num of points: "<<cloud->points.size()<<endl;
    cout<<"cloud height: "<<cloud->height<<endl;
    cout<<"cloud width: "<<cloud->width<<endl;
    if((counter%20) == 0){
        pcl::io::savePCDFileASCII ("/home/odroid/Desktop/output/output" + std::to_string(counter) + ".pcd", *cloud);
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg (*cloud, msg);
        pointcloud_pub.publish(msg);
    }
}
