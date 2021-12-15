#ifndef UTIL_H
#define UTIL_H

#include <geometry_msgs/Pose.h>   

struct Target {
    geometry_msgs::Pose pose; 
    int id = -1; // -1 means null id 
};

#endif


