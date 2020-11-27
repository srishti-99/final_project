# final_project

## To run RRT: 

Terminal 1: roscore 
Terminal 2: rosrun rviz rviz 
Terminal 3: rosrun rrt scripts/assignment.py 
Terminal 4: rostopic echo path_points 

This will display the RRT as its running and the path_points topic which its publishing to. 

## File structure and description: 

scripts/assignment.py: this contains the code that we took from the other repo for basically running RRT in rviz. We need to hook this up to gazebo and figure out a way to make the turtlebot be the robot instead of the robot marker they've created. This publishes to a topic called "/path_points" --> returns a list of poses that the turtle bot must follow to get to the target state. 

scripts/unicycle_controller.py: this is based on lab4 -- I've edited it to a large degree to subscribe to the path_points topic, take in the required points, and then try to navigate to them using some feedback control. THINGS TO DO: sync this with gazebo, FIND A WAY TO KEEP GETTING THE ROBOTS TRUE POSITION IN THE LOOP.  Also: i've created an "error_epsilon" variable which we need to tune so that the controller makes the robot go to each point in the path within some error range. 

scripts/viz.py: this i think creates all the vizualisation stuff you see in rviz when you run the code. 

scripts/tree.py: this creates the tree class which we're using to run the RRT stuff. 
