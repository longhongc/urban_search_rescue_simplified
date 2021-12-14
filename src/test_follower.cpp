#include <iostream>

#include <ros/ros.h>

#include "follower.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_test_node");
  ros::NodeHandle nh;

  Follower follower{&nh};  
  //follower.search_targets(); 
  //follower.reset_position(); 

  return 0;
}
