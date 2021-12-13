#include <iostream>

#include "rescue_bot.h"

Rescue_Bot::Rescue_Bot(ros::NodeHandle* nodehandle, Robot_Type robot_type, int number_of_targets) :
    m_nh{*nodehandle},
    m_type{robot_type} {


    if (m_type == Robot_Type::Explorer) {
        m_type_name = "explorer"; 
        m_start_position = {-4, 2.5}; 
        ROS_INFO("Robot initializing (%s)", m_type_name.c_str()); 
        ROS_INFO("Getting Targets Position... (%s)", m_type_name.c_str()); 
        get_targets_position(number_of_targets); 
    }
}

void Rescue_Bot::print_targets_position() {
    int count = 1; 
    for (auto &target: m_targets) {
        std::cout << "target " << count << ": "
              << "(" << target.x << ", " << target.y << ")" << std::endl; 
        count++; 
    }

}

void Rescue_Bot::search_targets() {
   for (int i=0; i<m_targets.size(); i++) {
        auto target_position = m_targets.at(i); 
        ROS_INFO("Sending goal to target %d (%s)", i+1, m_type_name.c_str());
        move_to_position(target_position); 
        ROS_INFO("Robot reached target %d (%s)", i+1, m_type_name.c_str());
        detect_aruco_marker(); 
    }
    ROS_INFO("Targets search finish (%s)", m_type_name.c_str());
}

void Rescue_Bot::reset_position() {
    ROS_INFO("Back to start position (%s)", m_type_name.c_str());
    move_to_position(m_start_position); 
    ROS_INFO("Reset position finish (%s)", m_type_name.c_str());
}

void Rescue_Bot::move_to_position(Position target_position){
    static MoveBaseClient movebase_client(std::string("/" + m_type_name + "/move_base").c_str(), true);
    while (!movebase_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for %s", m_type_name.c_str());
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

void Rescue_Bot::get_targets_position(int number_of_targets) {
    for (int i=1; i < number_of_targets+1; i++) {
        std::vector<double> target_pos; 
        std::string target_pos_param = "/aruco_lookup_locations/target_" + std::to_string(i); 
        if (m_nh.hasParam(target_pos_param)) {
            m_nh.getParam(target_pos_param, target_pos); 
            m_targets.emplace_back(Position{target_pos.at(0), target_pos.at(1)}); 
        }else {
            std::cout << "Target " << i << " position retrieving failed" << std::endl; 
        }

    }
}

void Rescue_Bot::fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    if (!msg->transforms.empty()) {
        ROS_INFO("Find marker");
        m_find_marker = true; 
        //broadcaster object
        // static tf2_ros::TransformBroadcaster br;
        // geometry_msgs::TransformStamped transformStamped;
        // //broadcast the new frame to /tf Topic
        // transformStamped.header.stamp = ros::Time::now();
        // transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        // transformStamped.child_frame_id = "marker_frame"; //name of the frame
        // transformStamped.transform.translation.x =
        // msg->transforms[0].transform.translation.x;,→
        // /*write the remaining code here*/
        // br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
    }
}

void Rescue_Bot::detect_aruco_marker(){
    static ros::Publisher vel_pub = m_nh.advertise<geometry_msgs::Twist>(std::string(m_type_name + "/cmd_vel").c_str(), 100); 
    static ros::Subscriber m_fiducial_sub = m_nh.subscribe("/fiducial_transforms", 100, &Rescue_Bot::fiducial_callback, this); 

    ros::Rate loop_rate(20);

    while (ros::ok() && !m_find_marker) {
        geometry_msgs::Twist msg; 
        msg.linear.x = 0.0; 
        msg.linear.y = 0.0; 
        msg.linear.z = 0.0; 
        msg.angular.x = 0.0; 
        msg.angular.y = 0.0; 
        msg.angular.z = 0.1; 

        vel_pub.publish(msg); 

        ros::spinOnce(); 

        loop_rate.sleep();
    }
    m_find_marker = false; 

}
