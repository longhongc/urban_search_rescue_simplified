#include <iostream>

#include <ros/ros.h>

#include "follower.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_node");
  ros::NodeHandle nh;

  Follower follower{&nh};  
  follower.print_targets_pose(); 
  follower.search_targets(); 
  follower.reset_pose(); 

  ros::shutdown(); 

  return 0;
}
