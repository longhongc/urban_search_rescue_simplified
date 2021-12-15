#include <iostream>
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>   
#include <std_srvs/Trigger.h>

#include "explorer.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* --------------------------------------------------------------------------*/
/**
 * @Brief Constructor for Explorer
 *
 * @Param nodehandle ros handler
 */
/* --------------------------------------------------------------------------*/
Explorer::Explorer(ros::NodeHandle* nodehandle) :
    m_nh{*nodehandle} {

    m_start_pose.position.x = -4; 
    m_start_pose.position.y = 2.5; 
    m_start_pose.orientation.w = 1.0; // the default orientation

    ROS_INFO("Robot initializing"); 
    ROS_INFO("Getting targets pose"); 
    get_targets_pose(); 
}

/* --------------------------------------------------------------------------*/
/**
 * @Brief Print the saved targets position for debugging
 */
/* --------------------------------------------------------------------------*/
void Explorer::print_targets_pose() {
    int count = 1; 
    for (auto &target: m_targets) {
        std::cout << "target " << count << ": "
              << "(" << target.pose.position.x << ", " << target.pose.position.y << ")" << std::endl; 
        count++; 
    }

}

/* --------------------------------------------------------------------------*/
/**
 * @Brief Search for the recoreded targets in sequence
 */
/* --------------------------------------------------------------------------*/
void Explorer::search_targets() {
   for (auto &target: m_targets) {
        auto target_pose = target.pose; 
        ROS_INFO("Sending goal to target %d", target.id);
        move_to_pose(target_pose); 
        ROS_INFO("Robot reached target %d", target.id);
        detect_aruco_marker(); 
    }
    ROS_INFO("Targets search finish");
}

/* --------------------------------------------------------------------------*/
/**
 * @Brief Move the explorer back to start point
 */
/* --------------------------------------------------------------------------*/
void Explorer::reset_pose() {
    ROS_INFO("Back to start pose");
    move_to_pose(m_start_pose); 
    ROS_INFO("Reset pose finish");
    ros::service::waitForService("start_rescue"); 
    std_srvs::Trigger start_rescue_srv; // call service to tell follower to rescue
    ros::service::call("start_rescue", start_rescue_srv); 
}

/* --------------------------------------------------------------------------*/
/**
 * @Brief Move explorer to a pose
 *
 * @Param target_pose a pose contains position and orientation
 */
/* --------------------------------------------------------------------------*/
void Explorer::move_to_pose(geometry_msgs::Pose target_pose){
    static MoveBaseClient movebase_client("/explorer/move_base", true);
    while (!movebase_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool goal_sent = false; 

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = target_pose;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (!goal_sent){
            movebase_client.sendGoal(goal);
            goal_sent = true;
        }
        if (movebase_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            break; 
        }
        loop_rate.sleep();
    }
}

/* --------------------------------------------------------------------------*/
/**
 * @Brief Getting the targets position from the ros param server
 */
/* --------------------------------------------------------------------------*/
void Explorer::get_targets_pose() {
    for (int i=1; i < m_number_of_targets+1; i++) {
        std::vector<double> target_position; 
        std::string target_pos_param = "/aruco_lookup_locations/target_" + std::to_string(i); 
        if (m_nh.hasParam(target_pos_param)) {
            m_nh.getParam(target_pos_param, target_position); 

            geometry_msgs::Pose target_pose; 
            target_pose.position.x = target_position.at(0); 
            target_pose.position.y = target_position.at(1); 
            target_pose.orientation.w = 1.0; 

            m_targets.emplace_back(Target{target_pose, i}); 

        }else {
            std::cout << "Target " << i << " position retrieving failed" << std::endl; 
        }

    }
}


/* --------------------------------------------------------------------------*/
/**
 * @Brief The callback function for subscriber of /fidicial_transforms
 *        Publish the targets pose on to tf server for follower to trace
 *        The targets pose is a pose with a distance of tolerance to the fiducial marker frame
 *
 * @Param msg /fidicial_transforms ros message
 */
/* --------------------------------------------------------------------------*/
void Explorer::fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    if (!msg->transforms.empty()) {
        ROS_INFO("Find marker");
        m_find_marker = true; // to inform the explorer that a marker has been detected 
        //broadcaster object
        geometry_msgs::TransformStamped transformStamped;

        //broadcast the new frame to /tf Topic
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        std::string target_name = "target_" + std::to_string(msg->transforms[0].fiducial_id); 
        ROS_INFO("Broadcasting: %s", target_name.c_str()); 
        transformStamped.child_frame_id = target_name; //name of the frame
        transformStamped.transform = msg->transforms[0].transform;
        double tolerance = 0.3; // the distance between the marker and target frame
                                // set tolerance because if the target frame is too close to the wall
                                // movebase cannot find valid plan
                                //
        m_marker_direction = (transformStamped.transform.translation.y > 0)? 1 : -1;  

        transformStamped.transform.translation.x *= tolerance;
        transformStamped.transform.translation.y *= tolerance;
        transformStamped.transform.translation.z *= tolerance;


        m_br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
    }
}

/* --------------------------------------------------------------------------*/
/**
 * @Brief  Slowly turn and detect arucu marker 
 *         After the fiducial subscriber find the marker on first time,  
 *         the robot will turn very slow for a small duration to get clearer marker pose data
 */
/* --------------------------------------------------------------------------*/
void Explorer::detect_aruco_marker(){
    m_find_marker = false;  

    static ros::Publisher vel_pub = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 100); 
    ros::Subscriber m_fiducial_sub = m_nh.subscribe("/fiducial_transforms", 100, &Explorer::fiducial_callback, this); 

    ros::Rate loop_rate(20);

    double turning_vel = 0.3;  
    double turned_angle = 0.0; 
    ros::Duration total_turning_time(0); 
    ros::Duration d(0);
    while (ros::ok()) {
        ros::Time begin = ros::Time::now();
        geometry_msgs::Twist msg; 
        msg.linear.x = 0.0; 
        msg.linear.y = 0.0; 
        msg.linear.z = 0.0; 
        msg.angular.x = 0.0; 
        msg.angular.y = 0.0; 
        msg.angular.z = turning_vel; 

        vel_pub.publish(msg); 

        ros::spinOnce(); 
        loop_rate.sleep();

        ros::Time end = ros::Time::now();

        total_turning_time += end - begin;
        turned_angle = total_turning_time.toSec() * turning_vel; 
        if(m_find_marker && turned_angle > 2 * M_PI){
            break; 
        }
        // decrease spining speed for better detection after first marker found
        // if(m_find_marker) {
        //     d += end - begin;
        //     // small delay time for better detection
        //     bool finish_delay = d.toSec() > 5; 
        //     turning_vel = 0.05; 
        //     if(finish_delay) {
        //         break; 
        //     }
        // }
    }
    m_find_marker = false; 
}
