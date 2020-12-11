# final_project

## To run project: 

Terminal 1: roscore

Terminal 2: rosrun rviz rviz 

Terminal 3: 
export ROBOT_INITIAL_POSE="-x -4 -y 4 -Y 0"

export RVIZ_INITIAL_POS="-4 4 0"

roslaunch starter turtlebot_world.launch world_file:=$(rospack find starter)/worlds/robsWorld.world


Terminal 4: 
rosrun rrt obstacle_creator.py

Terminal 5: 
rosrun rrt assignment.py

Terminal 6: 
rosrun rrt controller.py 

Terminal 7: 
rosrun rrt everything.py 

Terminal 8: 
rosrun starter initializer.py

Input the following parameters—> 
Robot starting x: -4 Robot starting y: 4 Block starting x: -3.5 Block starting y: 1.5 Target starting x: -2.5 Target starting y: -3.5

Also in rviz make sure the global frame is "map", add a Marker with topic "visualization_marker", add TF, robotmodel.

## To pull from github and set up build and devel
1. Delete the build and devel dirs inside workspace
2. Delete the CMakeLists.txt file inside workspace/src
3. Inside workspace/src run "catkin_init_workspace"
4. Inside workspace directory run "catkin_make"

## File structure and description: 

scripts/assignment.py: this contains the code that visualizes rrt running.

scripts/controller.py: this is based on lab4 -- I've edited it to a large degree to subscribe to the path_points topic, take in the required points, and then try to navigate to them using some feedback control. THINGS TO DO: sync this with gazebo, FIND A WAY TO KEEP GETTING THE ROBOTS TRUE POSITION IN THE LOOP.  Also: i've created an "error_epsilon" variable which we need to tune so that the controller makes the robot go to each point in the path within some error range. 

scripts/tree.py: this creates the tree class which we're using to run the rrt. 
