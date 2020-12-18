# final_project

## To run RRT: 

Terminal 1: roscore 

Terminal 2: rosrun rviz rviz 

Terminal 3: rosrun rrt scripts/assignment.py 

Terminal 4: rostopic echo path_points 

Terminal 5: export RVIZ_INITIAL_POS="-4 4 0"

export ROBOT_INITIAL_POSE="-x -4 -y 4 -Y 0"  
(this makes the turtlebot start at that initial position, where -Y stands for yaw)

In the SAME terminal, run:
roslaunch starter turtlebot_world.launch world_file:=$(rospack find starter)/worlds/robsWorld.world

Terminal 6: rosrun rrt scripts/controller.py 

This will display the RRT as its running and the path_points topic which its publishing to. 
The controller will subscribe to path_points and then run a turtlebot with the name "robot_0". We need to figure out how to spawn this turtlebot. I think the terminal 5 command should work? 

Also in rviz make sure the global frame is "map", add a Marker with topic "visualization_marker", add a TF for "robot_0".

## To pull from github and set up build and devel
1. Delete the build and devel dirs inside workspace
2. Delete the CMakeLists.txt file inside workspace/src
3. Inside workspace/src run "catkin_init_workspace"
4. Inside workspace directory run "catkin make"

## File structure and description: 

scripts/assignment.py: this contains the code that runs Optimized RRT. 
  Source code for basic RRT implementation and visualization: https://github.com/guaje/ros-rrt/tree/master/src/rrt/scripts 

scripts/controller.py: this is based on lab4.

scripts/tree.py: this creates the tree class which we're using to run the RRT stuff. 
