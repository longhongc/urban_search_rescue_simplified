#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <vector>
#include <queue>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>   
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
 
#include "util.h"

class Follower {
    public:
        Follower(ros::NodeHandle* nodehandle); 
        void print_targets_pose(); 
        void search_targets(); 
        void reset_pose();  


    private:
        void move_to_pose(geometry_msgs::Pose target_pose); 
        bool start_rescue(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);  
        void listen(tf2_ros::Buffer& tfBuffer); 
        void get_targets_pose(); 

        ros::NodeHandle m_nh;  
        ros::ServiceServer m_start_rescue_service; 
        bool m_start_rescue = false;

        static const int m_number_of_targets = 4; 
        std::array<std::queue<geometry_msgs::Pose>, m_number_of_targets> m_listened_data; 
        std::vector<Target> m_targets;  

        geometry_msgs::Pose m_start_pose; 

}; 


#endif


