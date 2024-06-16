Title: Task1: Autonomous Field Navigation

## Description
This packages farmland_navigation is used to autonomously navigate through the farmland using a complex navigation algorithm (using visual servoing), as the robot uses the center camera to detect the pegs( which are seen as obstacles and the green plants to avoid destroying if while navigating, by means of using image processing to filter out the pegs from the rest of the farmland model) and makes a decision to either turn left or right depending on the position of the peg with respect to the center camer. Hence, it moves away from the current peg position to avoid hitting it. This process is done throughout the whole navigation till it hits the goal location. 

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)
  

## Dependencies
Packages needed are: 
opencv: Open-source computer vision library.
```
    $ sudo apt-get install python3-opencv
```
- ROS2 packages including parc_robot_interfaces, sensor_msgs, rclpy, std_msgs, cv_bridge.
- dependence are located in the package.xml
```
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>parc_robot_interfaces</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
```
  
- `ros2_control`: ROS packages including controller interface, controller manager, hardware interface etc.
    `$ sudo apt-get install ros-humble-ros2-control`

## Package Overview

* This section details the custom ROS2 packages developed for our project:

* parc_robot_bringup (./parc_robot_bringup/): This package houses configuration files, the simulated world, scripts, and launch files responsible for bringing up the PARC robot for both Task 1 and Task 2.

* parc_robot_description (./parc_robot_description/): This package contains the URDF (Unified Robot Description Format) description files for the PARC robot, its sensors, and launch files for the robot state publisher. URDF is an XML format used to describe the robot's physical structure and capabilities.

* parc_robot_interfaces (./parc_robot_interfaces/): This package includes custom message definitions used in our project for communication between various components. These custom messages likely facilitate data exchange specific to our robot's functionalities.


## Task 1

In Task 1, we used a camera for visual servoing, creating a model to detect pegs, which guided the robot to avoid them. Additionally, we utilized point cloud data from the camera for navigation and turning. This approach ensured the robot could effectively navigate the field, avoiding obstacles and staying on course.

Command to run the solution:
```
  source ~/ros2_ws/install/setup.bash
` ros2 run farmland_navigation task1_solution `
```
## Installation
Steps to install and set up for your project.

'''markdown

1. Inside your workspace(ros2_ws) create a new ROS2 python package called farmland_navigation using -
```
ros2 "pkg create farmland_navigation --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs"
```

3. when you have your task1_solution.py code in your farmland_navigation folder inside the farmland_navigation package, check when the code is executable by setting up the setup.py console.script like this:
```
    entry_points={
        'console_scripts': [
            'task1_solution = farmland_navigation.task1_solution:main',
        ],
    },
```

5. If all is setup and everything is in the right path, build the workspace again using colcon build and source ~/ros2_ws/install/setup.bash after. Then run the code
```
    source ~/ros2_ws/install/setup.bash
    ros2 run farmland_navigation task1_solution
```

7. Robot would start navigating through the farmland. 


## Challenges Faced
(Task 1)
- We were nto able to create a map of the environment due to wild lidar readings. causing the map not to form properly
- Even while using servoing for this task, we saw that the camera positions on the robotmodel was not set properly for visualization. for instance, the front camera was also covering some part of the robot blocking visualization.
- The responds is not the best. for instance, turning the robot was hard, because he had to vary alot of angles and it was slow.
