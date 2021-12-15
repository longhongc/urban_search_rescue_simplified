/**
 * @file explorer.h
 * @Brief THe explorer class  
 */

#ifndef EXPLORER_BOT_H
#define EXPLORER_BOT_H

#include <string>
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>   
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include "util.h"

/* --------------------------------------------------------------------------*/
/**
 * @Brief  The explorer class that provides functions for the robot to perform searching mission
 *         Searching mission is to find all the targets, locate them, and broadcast the pose to other robot
 */
/* --------------------------------------------------------------------------*/
class Explorer {
    public:
        Explorer(ros::NodeHandle* nodehandle); 
        void print_targets_pose(); 
        void search_targets(); 
        void reset_pose();  

    private:
        void move_to_pose(geometry_msgs::Pose target_pose); 
        void get_targets_pose(); 
        void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg); 
        void detect_aruco_marker(); 

        ros::NodeHandle m_nh;  
        geometry_msgs::Pose m_start_pose; 

        const int m_number_of_targets = 4; 
        std::vector<Target> m_targets; // use vector instead of array to be more flexible when target can't be detected

        bool m_find_marker = false;  

        tf2_ros::TransformBroadcaster m_br;
}; 


#endif


