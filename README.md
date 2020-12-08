# final_project

## To run RRT: 

Terminal 1: roscore 

Terminal 2: rosrun rviz rviz 

Terminal 3: rosrun rrt scripts/assignment.py 

Terminal 4: rostopic echo path_points 

Terminal 5: roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/robsWorld.world

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

scripts/assignment.py: this contains the code that we took from the other repo for basically running RRT in rviz. We need to hook this up to gazebo and figure out a way to make the turtlebot be the robot instead of the robot marker they've created. This publishes to a topic called "/path_points" --> returns a list of poses that the turtle bot must follow to get to the target state. 

scripts/controller.py: this is based on lab4 -- I've edited it to a large degree to subscribe to the path_points topic, take in the required points, and then try to navigate to them using some feedback control. THINGS TO DO: sync this with gazebo, FIND A WAY TO KEEP GETTING THE ROBOTS TRUE POSITION IN THE LOOP.  Also: i've created an "error_epsilon" variable which we need to tune so that the controller makes the robot go to each point in the path within some error range. 

scripts/viz.py: this i think creates all the vizualisation stuff you see in rviz when you run the code. 

scripts/tree.py: this creates the tree class which we're using to run the RRT stuff. 

scripts/demo.py: this i think is irrelevant to us but im not deleting it yet. I'm going to clean up the code later will delete it then. 
