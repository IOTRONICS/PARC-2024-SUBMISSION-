Title: Task2: Crop Yield Estimation using YOLOv8

## Description
This packages tomatoes_detection1 is used to detect tomatoes in the field using object detection with the application of YOLOv8. When the robot detects the tomatoes, it counts each tomato detected on the field  once and at the end of the task it publishes the total count of the left and right camera/image to the CropYield msg and stops. 

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

1. Inside your workspace(ros2_ws) create a new ROS2 python package called tomatoes_detection1 using - ros2 pkg create tomatoes_detection1 --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs

- add all necessary depedencies shown below in the package.xml:
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>parc_robot_interfaces</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

2. when you have your task2_solution.py code in your tomatoes_detection1 folder inside the tomatoes_detection1 package, check when the code is executable by setting up the setup.py console.script like this:
entry_points={
        'console_scripts': [
            'task2_solution = tomatoes_detection1.task2_solution:main',
        ],
    },

3. In the tomatoes_detection1 package, there is a "runs" folder and a "data" folder added to it. these folders are used for the yolov8 operation.

4. If all is setup and everything is in the right path, build the workspace again using colcon build and source ~/ros2_ws/install/setup.bash after. Then run the code " ros2 run tomatoes_detection1 task2_solution" 

5. Robot would start detecting the tomatoes when camera data is availabe. 

