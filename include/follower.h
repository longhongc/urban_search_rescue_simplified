#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>   
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>

#include <util.h>

class Follower {
    public:
        Follower(ros::NodeHandle* nodehandle); 
        void print_targets_position(); 
        void search_targets(); 
        void reset_position();  
        void move_to_position(Position target_position); 


    private:
        void listen(tf2_ros::Buffer& tfBuffer); 
        void get_targets_position(); 

        ros::NodeHandle m_nh;  

        int m_number_of_targets = 4; 
        std::vector<Position> m_targets;  

        Position m_start_position = {-4, 3.5}; 

        double m_tolerance = 0.4; 
}; 


#endif


