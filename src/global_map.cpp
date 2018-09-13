#include "global_map.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

bool GlobalMap::isTooClose() {
    bool isTooCloseFlag = true;
    float dis = pow(transform_prev.translation.x - transform_curr.translation.x, 2) 
    + pow(transform_prev.translation.y - transform_curr.translation.y, 2);
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

void GlobalMap::sparsification(pcl::PointCloud<pcl::PointXYZ>& dense_cloud){
    pcl::PointCloud<pcl::PointXYZ> copy = dense_cloud;
    dense_cloud.clear();
    int idx_keep = int(1/keep_prob);
    for(int i = 0; i < copy.points.size(); ++i) {
        if( i%idx_keep == 0)
            dense_cloud.push_back(copy.points[i]);
    }
}

void GlobalMap::dim_reduction(pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d, pcl::PointCloud<pcl::PointXYZ>& cloud_3d) {
    cloud_2d.reset(new (pcl::PointCloud<pcl::PointXY>));
    for(int i = 0; i < cloud_3d.points.size(); ++i){
        pcl::PointXY temp;
        temp.x = cloud_3d.points[i].x;
        temp.y = cloud_3d.points[i].y;
        cloud_2d->push_back(temp);
    }
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXY>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXY>);

    pcl::fromROSMsg (*cloud_msg, *dense_cloud);

    sparsification(*dense_cloud);

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = transform_prev.translation.x;
    trans(1, 3) = transform_prev.translation.y;
    pcl::transformPointCloud (*dense_cloud, *transformed_cloud, trans);

    dim_reduction(flattened_cloud, *transformed_cloud);

    cloud_queue.push(flattened_cloud);
    if(cloud_queue.size() >= queue_size) {
        pcl::PointCloud<pcl::PointXY>::Ptr temp = cloud_queue.front();
        temp.reset();
        cloud_queue.pop();
    }
    counter++;
    cout<<"counter: "<<"\t"<<"\t"<<"\t"<<"\t"<<"\t"<<counter<<endl;
    //cout<<"num of points: "<<cloud->points.size()<<endl;
    if( (counter%10) == 0 ) {//for testing
        queue< pcl::PointCloud<pcl::PointXY>::Ptr > clone_queue = cloud_queue;
        pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
        while(!clone_queue.empty()){
            pcl::PointCloud<pcl::PointXY>::Ptr temp_cloud = clone_queue.front();
            *cloud += *temp_cloud;
            clone_queue.pop();
        } 
        //pcl::io::savePCDFileASCII ("/home/odroid/Desktop/output/output" + std::to_string(counter) + ".pcd", *cloud);
        pcl::io::savePCDFileASCII ("/home/squareone/Desktop/throne/icp_2d_test/output/output" + std::to_string(counter) + ".pcd", *cloud);
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg (*cloud, msg);
        pointcloud_pub.publish(msg);
        cloud.reset();
    }
}
