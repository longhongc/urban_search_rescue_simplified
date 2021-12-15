#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <string>
#include <vector>
#include <queue>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>   
#include <geometry_msgs/Pose.h>   
#include <geometry_msgs/Point.h>   
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>

#include <ros/ros.h>
 
#include <util.h>

class Follower {
    public:
        Follower(ros::NodeHandle* nodehandle); 
        void print_targets_pose(); 
        void search_targets(); 
        void reset_pose();  
        void move_to_pose(geometry_msgs::Pose target_pose); 


    private:
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


