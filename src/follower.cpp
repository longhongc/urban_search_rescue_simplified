#include <iostream>

#include "follower.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Follower::Follower(ros::NodeHandle* nodehandle) :
    m_nh{*nodehandle} {

    ROS_INFO("Robot initializing"); 
    ROS_INFO("Getting Targets Position..."); 
    get_targets_position(); 
}

void Follower::print_targets_position() {
    int count = 1; 
    for (auto &target: m_targets) {
        std::cout << "target " << count << ": "
              << "(" << target.x << ", " << target.y << ")" << std::endl; 
        count++; 
    }

}

void Follower::search_targets() {
   for (int i=0; i<m_targets.size(); i++) {
        auto target_position = m_targets.at(i); 
        target_position.x += m_tolerance; 
        target_position.y += m_tolerance; 
        ROS_INFO("Sending goal to target %d", i);
        move_to_position(target_position); 
        ROS_INFO("Robot reached target %d", i);
    }
    ROS_INFO("Targets search finish");
}

void Follower::reset_position() {
    ROS_INFO("Back to start position");
    move_to_position(m_start_position); 
    ROS_INFO("Reset position finish");
}

void Follower::move_to_position(Position target_position){
    static MoveBaseClient movebase_client("/follower/move_base", true);
    while (!movebase_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool goal_sent = false; 

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = target_position.x;
    goal.target_pose.pose.position.y = target_position.y;
    goal.target_pose.pose.orientation.w = 1.0;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (!goal_sent){
            movebase_client.sendGoal(goal);//this should be sent only once
            goal_sent = true;
        }
        if (movebase_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            break; 
        }
        loop_rate.sleep();
    }
}

void Follower::listen(tf2_ros::Buffer& tfBuffer) {
    for(int i=0; i < m_number_of_targets; i++){
        geometry_msgs::TransformStamped transformStamped;
        try {
          std::string target_frame = "target_" + std::to_string(i); 
          transformStamped = tfBuffer.lookupTransform("map", target_frame.c_str(), ros::Time(0));
          auto trans_x = transformStamped.transform.translation.x;
          auto trans_y = transformStamped.transform.translation.y;
          auto trans_z = transformStamped.transform.translation.z;
      
          ROS_INFO_STREAM("Position in map frame: ["
            << trans_x << ","
            << trans_y << ","
            << trans_z << "]"
          );
        }
        catch (tf2::TransformException& ex) {
          //ROS_WARN("%s", ex.what());
          ros::Duration(0.05).sleep();
        }
    }
}

void Follower::get_targets_position() {
    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(20);
  
    while (ros::ok()) {
        listen(tfBuffer);

        loop_rate.sleep();
    }
}
