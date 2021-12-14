#include <iostream>

#include <ros/ros.h>

#include "rescue_bot.h"

void listen0(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "target_0", ros::Time(0));
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
    ROS_WARN("%s", ex.what());
    ros::Duration(0.1).sleep();
  }
}

void listen1(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "target_1", ros::Time(0));
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
    ROS_WARN("%s", ex.what());
    ros::Duration(0.1).sleep();
  }
}

void listen2(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "target_2", ros::Time(0));
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
    ROS_WARN("%s", ex.what());
    ros::Duration(0.1).sleep();
  }
}

void listen3(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "target_3", ros::Time(0));
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
    ROS_WARN("%s", ex.what());
    ros::Duration(0.1).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_follower");
  ros::NodeHandle nh;

  //Rescue_Bot explorer{&nh, Robot_Type::Explorer};  
  //explorer.search_targets(); 
  //explorer.reset_position(); 

  tf2_ros::Buffer tfBuffer0;
  tf2_ros::Buffer tfBuffer1;
  tf2_ros::Buffer tfBuffer2;
  tf2_ros::Buffer tfBuffer3;

  tf2_ros::TransformListener tfListener0(tfBuffer0);
  tf2_ros::TransformListener tfListener1(tfBuffer1);
  tf2_ros::TransformListener tfListener2(tfBuffer2);
  tf2_ros::TransformListener tfListener3(tfBuffer3);
  ros::Rate loop_rate(200);

  while (ros::ok()) {
    
    listen0(tfBuffer0);
    listen1(tfBuffer1);
    listen2(tfBuffer2);
    listen2(tfBuffer3);
    loop_rate.sleep();
  }

  return 0;
}
