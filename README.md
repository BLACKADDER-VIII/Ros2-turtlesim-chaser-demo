# Turtlesim Demo Project

## Building the Project:
Go the the directory /turtle\_pkg/,
it should contain package.xml, setup.py and a subdirectory turtle\_pkg/ (same name)
Use `colcon build`  
It should create an install and build folder. The project has been built.

## Running the project:
Open 4 terminals in the directory /turtle\_pkg
In each terminal use `source install/setup.bash`
Then:  

Terminal 1: Run `ros2 run turtlesim turtlesim\_node`  
Terminal 2: Run `ros2 run turtle\_pkg car\_ahead`  
Terminal 3: Run `ros2 run turtle\_pkg car\_behind`  
Terminal 4: Run `ros2 run turtle\_pkg car\_emerg`  

This should run the turtlesim window with the three turtles. Terminal 4 will stop spinning the node once emergency vehicle reaches its destination. The other terminals must be stopped manually.

Observations:
Average time to completion of emergency vehicle: 5.2s
Average close collisions: 6
Successful yield events: 34
