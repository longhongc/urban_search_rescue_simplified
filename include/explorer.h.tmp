#ifndef EXPLORER_BOT_H
#define EXPLORER_BOT_H

#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>   
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Trigger.h>

#include <ros/ros.h>

#include <util.h>

class Explorer {
    public:
        Explorer(ros::NodeHandle* nodehandle); 
        void print_targets_position(); 
        void search_targets(); 
        void reset_position();  
        void move_to_position(Position target_position); 


    private:
        void get_targets_position(); 
        void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg); 
        void detect_aruco_marker(); 

        ros::NodeHandle m_nh;  

        int m_number_of_targets = 4; 
        std::vector<Position> m_targets;  

        Position m_start_position = {-4, 2.5}; 
        bool m_find_marker = false;  

        tf2_ros::TransformBroadcaster m_br;
}; 


#endif


