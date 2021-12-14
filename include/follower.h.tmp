#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <string>
#include <vector>
#include <queue>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>   
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Trigger.h>

#include <ros/ros.h>

#include <util.h>

struct Targets {
    Position pos; 
    int id = -1; // -1 means null id 
}; 

class Follower {
    public:
        Follower(ros::NodeHandle* nodehandle); 
        void print_targets_position(); 
        void search_targets(); 
        void reset_position();  
        void move_to_position(Position target_position); 


    private:
        bool start_rescue(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);  
        void listen(tf2_ros::Buffer& tfBuffer); 
        void get_targets_position(); 

        ros::NodeHandle m_nh;  
        ros::ServiceServer m_start_rescue_service; 
        bool m_start_rescue = false;

        static const int m_number_of_targets = 4; 
        std::array<std::queue<Position>, m_number_of_targets> m_listened_data; 
        std::vector<Targets> m_targets;  

        Position m_start_position = {-4, 3.5}; 

}; 


#endif


