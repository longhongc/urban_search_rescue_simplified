#include <iostream>

#include <ros/ros.h>

#include "rescue_bot.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  Rescue_Bot explorer{&nh, Robot_Type::Explorer};  
  explorer.search_targets(); 
  explorer.reset_position(); 

  return 0; 
}
