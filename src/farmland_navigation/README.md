Title: Task1: Autonomous Field Navigation

## Description
This packages farmland_navigation is used to autonomously navigate through the farmland using a complex navigation algorithm, as the robot uses the center camera to detect the pegs( which are seen as obstacles, by means of using image processing to filter out the pegs from the rest of the farmland model) and makes a decision to either turn left or right depending on the position of the peg with respect to the center camer. Hence, it moves away from the current peg position to avoid hitting it. This process is done throughout the whole navigation till it hits the goal location. 

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)


## Installation
Steps to install and set up your project.

'''markdown

1. Inside your workspace(ros2_ws) create a new ROS2 python package called farmland_navigation using - ros2 "pkg create farmland_navigation --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs"

- add all necessary depedencies shown below in the package.xml:
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>parc_robot_interfaces</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

2. when you have your task1_solution.py code in your farmland_navigation folder inside the farmland_navigation package, check when the code is executable by setting up the setup.py console.script like this:
    entry_points={
        'console_scripts': [
            'roi = farmland_navigation.task1_solution:main',
        ],
    },

3. If all is setup and everything is in the right path, build the workspace again using colcon build and source ~/ros2_ws/install/setup.bash after. Then run the code " ros2 run farmland_navigation roi" 

4. Robot would start navigating through the farmland. 

