#include <iostream>

#include <ros/ros.h>

#include "explorer.h"

 
int main(int argc, char** argv) {
  ros::init(argc, argv, "explorer_test_node");
  ros::NodeHandle nh;

  Explorer explorer{&nh};  
  explorer.search_targets(); 
  explorer.reset_position(); 

  return 0; 
}
