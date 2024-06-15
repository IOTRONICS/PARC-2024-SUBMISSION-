# TEAM IOTRONICS : PARC Engineers League 

## Introduction

Overview of the Competition Task

The PARC 2024 competition involves developing software for the PARC AgRobot to autonomously navigate a simulated tomato field (Task 1) and estimate crop yield using computer vision techniques (Task 2).
Agricultural robotics enhances efficiency, productivity, and precision in farming. In Africa, these technologies can address labor shortages, optimize resource use, and improve crop yields, contributing significantly to food security and sustainable agricultural practices. Implementing such innovations can revolutionize farming methods, making them more resilient and adaptable to changing environmental conditions.

**Team Country:** Nigeira

**Team Member Names:**

* Samuel Oyefusi  (Team Leader)
* Oyefusi Samuel - oluwakorede.oyefusi@pau.edu.ng
* Onuoha Christian - Christian.onuoha@pau.edu.ng
* Israel Afolabi - Israel.afolabi@pau.edu.ng
* Mayowa Oluyoye - oluyoye22@gmail.com
* Amarachi Alu-Ewah- amarachi.alu-ewah@pau.edu.ng
* AFOLABI Ibukunoluwa- afosutech24@gmail.com
* George Uwagbale - uwagbalegeorge@gmail.com
* Chima Okwuokei - chima.okwuokei@pau.edu.ng
  

## Dependencies


## Package Overview

This section details the custom ROS packages developed for our project:

parc_robot_bringup (./parc_robot_bringup/): This package houses configuration files, the simulated world, scripts, and launch files responsible for bringing up the PARC robot for both Task 1 and Task 2.

parc_robot_description (./parc_robot_description/): This package contains the URDF (Unified Robot Description Format) description files for the PARC robot, its sensors, and launch files for the robot state publisher. URDF is an XML format used to describe the robot's physical structure and capabilities.

parc_robot_interfaces (./parc_robot_interfaces/): This package includes custom message definitions used in our project for communication between various components. These custom messages likely facilitate data exchange specific to our robot's functionalities.

**Packages needed are:** 

* `ros2_control`: ROS packages including controller interface, controller manager, hardware interface etc.

    * `$ sudo apt-get install ros-humble-ros2-control`

* `opencv`: Open-source computer vision library.
    pip install ultralytics 


    * `$ sudo apt-get install python3-opencv`

## Task 1

In Task 1, we used a camera for visual servoing, creating a model to detect pegs, which guided the robot to avoid them. Additionally, we utilized point cloud data from the camera for navigation and turning. This approach ensured the robot could effectively navigate the field, avoiding obstacles and staying on course.

Command to run the solution:
```
ros2 run farmland_navigation roi
```
## Task 2
We used OpenCV and YOLOv8 for tomato detection through object recognition. OpenCV handled the computer vision aspects, converting camera data and potentially filtering redundant detections. YOLOv8, trained on tomato data from the simulation environment (using Roboflow and ROS2 bags), performed the core object detection. This YOLOv8 model offered high accuracy but required a decent GPU for optimal performance.

Command to run the solution:

` ros2 run tomatoes_detection1 task2_solution `

## Challenges Faced
Challenges Faced
Internet connectivity issues.
Insufficient GPU for image processing tasks.
Difficulty in distinguishing closely packed tomatoes, resembling grape clusters.
Misidentification of green tomatoes as weeds.
Task 2
For Task 2, detecting tomatoes was challenging due to their tendency to cluster together, complicating image processing. Green tomatoes occasionally confused the model, causing it to misidentify them as weeds.--------------------------------------------------
