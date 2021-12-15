# Urban Search and Rescue
This is the final project of ENPM809Y. We simulated a simplified urban search and rescue mission.   
There are two robots. The first robots is called the explorer and it is reponsible for searching all the targets.   
It will then broadcast the tf frame of all targets. The other robot is the follower. It listens to the broadcasted tf and start rescue mission.  

<img src=videos/search_and_rescue_gazebo.gif width="500" height="350" />   
<img src=videos/search_and_rescue_rviz.gif width="500" height="350" />   

## Enviconment
Ubuntu 20.04
ROS noetic

## Dependencies
Install the dependencies script  
```
cd script
sh install.bash
```

## Build
Clone the project in src folder then build with catkin tool
```
cd {ros_workspace}/src
git clone https://github.com/longhongc/urban_search_rescue_simplified.git
cd {ros_workspace} && catkin_make (or catkin build)
```

## Run
Start the S&R RVIZ and Gazebo simulation 
```
roslaunch final_project multiple_robots.launch
```

Start the search and rescue mission
```
roslaunch final_project search_and_rescue.launch
```


