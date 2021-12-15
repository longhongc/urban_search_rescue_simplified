#include <iostream>

#include <ros/ros.h>

#include "explorer.h"

 
int main(int argc, char** argv) {
  ros::init(argc, argv, "explorer_node");
  ros::NodeHandle nh;

  Explorer explorer{&nh};  
  explorer.search_targets(); 
  explorer.reset_pose(); 

  ros::shutdown(); 

  return 0; 
}
