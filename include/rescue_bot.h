#ifndef RESCUE_BOT_H
#define RESCUE_BOT_H

#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>   
#include <fiducial_msgs/FiducialTransformArray.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Position{
    double x; 
    double y; 
}; 

enum class Robot_Type{Explorer}; 

class Rescue_Bot
{
    public:
    Rescue_Bot(ros::NodeHandle* nodehandle, Robot_Type type, int number_of_targets=4); 
    void print_targets_position(); 
    void search_targets(); 
    void reset_position();  
    void move_to_position(Position target_position); 


    private:
    void get_targets_position(int number_of_targets); 
    void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg); 
    void detect_aruco_marker(); 

    ros::NodeHandle m_nh;  

    Robot_Type m_type; 
    std::string m_type_name; 
    std::vector<Position> m_targets;  

    Position m_start_position; 
    bool m_find_marker = false;  

    //ros::Publisher m_velocity_publisher;
}; 


#endif


