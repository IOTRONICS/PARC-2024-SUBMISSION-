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

Include a brief description of your approach to the solution (*This should be only 5-7 sentences*).

Write the command required to run your solution. Should be in this format: <br>
` ros2 run <your-package-name> task2_solution.py `

## Challenges Faced
Challenges Faced
Internet connectivity issues.
Insufficient GPU for image processing tasks.
Difficulty in distinguishing closely packed tomatoes, resembling grape clusters.
Misidentification of green tomatoes as weeds.
Task 2
For Task 2, detecting tomatoes was challenging due to their tendency to cluster together, complicating image processing. Green tomatoes occasionally confused the model, causing it to misidentify them as weeds.--------------------------------------------------
