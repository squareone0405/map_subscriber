#include "ros/ros.h"
#include <iostream>

#include "global_map.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "map_subscriber");
    ros::NodeHandle n;
    GlobalMap gMap(n);
    gMap.start();
    return 0;
}