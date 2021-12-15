#include <iostream>

#include "follower.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Follower::Follower(ros::NodeHandle* nodehandle) :
    m_nh{*nodehandle} {

    m_start_pose.position.x = -4; 
    m_start_pose.position.y = 3.5; 
    m_start_pose.orientation.w = 1.0; 

    m_start_rescue_service = m_nh.advertiseService("start_rescue", &Follower::start_rescue, this); 

    ROS_INFO("Robot initializing"); 
    ROS_INFO("Getting targets pose"); 

    get_targets_pose(); 
}

bool Follower::start_rescue(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    res.success = true; 
    res.message = "Follower start rescue"; 
    ROS_INFO("Follower start rescue"); 
    m_start_rescue = true; 

    return true; 
}

void Follower::print_targets_pose() {
    for (auto &target: m_targets) {
        std::cout << "target " << target.id << ": "
              << "(" << target.pose.position.x << ", " << target.pose.position.y << ")" << std::endl; 
    }

}

void Follower::search_targets() {
   for (auto &target: m_targets) {
        auto target_pose = target.pose; 
        ROS_INFO("Sending goal to target %d", target.id);
        move_to_pose(target_pose); 
        ROS_INFO("Robot reached target %d", target.id);
    }
    ROS_INFO("Targets search finish");
}

void Follower::reset_pose() {
    ROS_INFO("Back to start pose");
    move_to_pose(m_start_pose); 
    ROS_INFO("Reset pose finish");
}

void Follower::move_to_pose(geometry_msgs::Pose target_pose){
    static MoveBaseClient movebase_client("/follower/move_base", true);
    while (!movebase_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool goal_sent = false; 

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position = target_pose.position;
    goal.target_pose.pose.orientation = target_pose.orientation;
    //goal.target_pose.pose.orientation.w = 1.0;

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
          geometry_msgs::Pose target_pose; 
          target_pose.position.x = trans_x; 
          target_pose.position.y = trans_y; 
          target_pose.position.z = trans_z; 
          target_pose.orientation = transformStamped.transform.rotation; 

          m_listened_data.at(i).push(target_pose); 
          if(m_listened_data.at(i).size() > 20){
              m_listened_data.at(i).pop(); 
          }
        }
        catch (tf2::TransformException& ex) {
          std::cout << "." << std::flush; 
          ros::Duration(0.05).sleep();
        }
    }
}

void Follower::get_targets_pose() {
    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(20);
  
    while (ros::ok()) {
        listen(tfBuffer);

        ros::spinOnce(); 
        loop_rate.sleep();
        if(m_start_rescue){
            break; 
        }
    }
    
    // calculate the average of listened data
    for(int i=0; i < m_number_of_targets; i++){
        int num_of_data = m_listened_data.at(i).size(); 
        if(num_of_data == 0){
            continue; 
        }

        double avg_x = 0.0; 
        double avg_y = 0.0; 
        double avg_z = 0.0; 

        
        geometry_msgs::Quaternion quat_msg; 
        
        while(!m_listened_data.at(i).empty()){
            avg_x += m_listened_data.at(i).front().position.x; 
            avg_y += m_listened_data.at(i).front().position.y; 
            quat_msg = m_listened_data.at(i).front().orientation;
            
            m_listened_data.at(i).pop(); 
        }

        avg_x = avg_x / num_of_data; 
        avg_y = avg_y / num_of_data; 

        tf2::Quaternion quat_tf;
        tf2::fromMsg(quat_msg, quat_tf);
        double roll{};
        double pitch{};
        double yaw{};

        tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

        quat_tf.setRPY(0, 0, yaw+1.57); 
        quat_msg = tf2::toMsg(quat_tf);

        Target target; 
        target.pose.position.x = avg_x; 
        target.pose.position.y = avg_y; 
        target.pose.position.z = avg_z; 
        target.pose.orientation = quat_msg; 

        target.id = i; 

        m_targets.emplace_back(target); 
    }
}
