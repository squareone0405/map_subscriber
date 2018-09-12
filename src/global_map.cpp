#include "global_map.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

bool GlobalMap::isTooClose() {
    bool isTooCloseFlag = true;
    float dis = pow(transform_prev.translation.x - transform_curr.translation.x, 2) 
    + pow(transform_prev.translation.y - transform_curr.translation.y, 2) 
    + pow(transform_prev.translation.z - transform_curr.translation.z, 2);
    cout<<"dis: "<<dis<<endl;
    if(dis > 0.0025){
        isTooCloseFlag = false;
    }
    float qx = transform_curr.rotation.x;
    float qy = transform_curr.rotation.y;
    float qz = transform_curr.rotation.z;
    float qw = transform_curr.rotation.w;
    float roll_new = std::atan2(2 * (qw * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
    float pitch_new = std::asin(2 * (qw * qy - qx * qz));
    float yaw_new = std::atan2(2 * (qw * qz + qx * qy), (1 - 2 * (qz * qz + qy * qy)));
    float dis_angle = abs(roll_new - roll) + abs(pitch_new - pitch) + abs(yaw_new - yaw);
    cout<<"dis_angle: "<<dis_angle<<endl;
    if(dis_angle > M_PI / 36){
        isTooCloseFlag = false;
    } 
    if(!isTooCloseFlag){
        transform_prev = transform_curr;
        roll = roll_new;
        pitch = pitch_new;
        yaw = yaw_new;
    }
    return isTooCloseFlag;
}

void GlobalMap::tf_callback(const tf::tfMessage::ConstPtr &tf_msg){
    auto transforms = tf_msg->transforms;
    auto transform_new = transforms[0].transform;
    if(transforms[0].child_frame_id != string("curr_position"))
        return;
    transform_curr = transform_new;
}

void GlobalMap::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    if(isTooClose()){
        cout<<"reject! no move!"<<endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *temp_cloud);
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = transform_prev.translation.x;
    trans(1, 3) = transform_prev.translation.y;
    trans(2, 3) = transform_prev.translation.z;
    cout<< transform_prev.translation.x << '\t' << transform_prev.translation.y << '\t' << transform_prev.translation.z <<endl;
    pcl::transformPointCloud (*temp_cloud, *transformed_cloud, trans);
    cloud_queue.push(transformed_cloud);
    if(cloud_queue.size() >= queue_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp = cloud_queue.front();
        delete temp.get();
        cloud_queue.pop();
    }
    counter++;
    cout<<"counter: "<<counter<<endl;
    //cout<<"num of points: "<<cloud->points.size()<<endl;
    if( (counter%10) == 0 ){//for testing
        queue< pcl::PointCloud<pcl::PointXYZ>::Ptr > clone_queue = cloud_queue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        while(!clone_queue.empty()){
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = clone_queue.front();
            *cloud += *temp_cloud;
            clone_queue.pop();
        } 
        pcl::io::savePCDFileASCII ("/home/odroid/Desktop/output/output" + std::to_string(counter) + ".pcd", *cloud);
        //pcl::io::savePCDFileASCII ("/home/squareone/Desktop/output/output" + std::to_string(counter) + ".pcd", *cloud);
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg (*cloud, msg);
        pointcloud_pub.publish(msg);
        delete cloud.get();
    }
}
