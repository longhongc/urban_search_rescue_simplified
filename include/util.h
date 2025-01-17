/**
 * @file util.h
 * @Brief Utility functions for other files  
 */
#ifndef UTIL_H
#define UTIL_H

#include <geometry_msgs/Pose.h>   

/* --------------------------------------------------------------------------*/
/**
 * @Brief Information of a target on map 
 */
/* --------------------------------------------------------------------------*/
struct Target {
    geometry_msgs::Pose pose; 
    int id = -1; // -1 means null id 
};

#endif


