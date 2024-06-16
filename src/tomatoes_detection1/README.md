Title: Task2: Crop Yield Estimation using YOLOv8 and Opencv

## Description
This packages tomatoes_detection1 is used to detect tomatoes in the field using object detection with the application of YOLOv8 and computer vision using Opencv. When the robot detects the tomatoes using  a trained model(best.pt) that has been annotated, it counts each tomato detected on the field  once and at the end of the task it publishes the total count of the left and right camera/image to the CropYield msg and stops. 

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

## Dependencies
Packages needed are: 
To is to be able to successfully run the tomatoes_detection1 package to be able to use YOLOv8. Install in the tomatoes_detection1 directory.
```
pip:  pip install ultralytics 
```
opencv: Open-source computer vision library.
```
    $ sudo apt-get install python3-opencv
```
  
- ROS2 packages including parc_robot_interfaces, sensor_msgs, ament_index_python, rclpy, std_msgs, cv_bridge.
  
- dependence are located in the package.xml
```
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>parc_robot_interfaces</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>ament_index_python</depend>
```
  
- `ros2_control`: ROS packages including controller interface, controller manager, hardware interface etc.
    `$ sudo apt-get install ros-humble-ros2-control`


## Task 2
We used OpenCV and YOLOv8 for tomato detection through object recognition. OpenCV handled the computer vision aspects, converting camera data and potentially filtering redundant detections. YOLOv8, trained on tomato data from the simulation environment (using Roboflow and ROS2 bags), performed the core object detection. This YOLOv8 model offered high accuracy but required a decent GPU for optimal performance.

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
```
entry_points={
        'console_scripts': [
            'task2_solution = tomatoes_detection1.task2_solution:main',
        ],
    },
```

4. In the tomatoes_detection1 package, there is a "runs" folder and a "data" folder added to it. these folders are used for the yolov8 operation.
5. With the use of YOLOv8, we had to construct the path to the YOLO model weights by creating a shared directory in the task2_solution.py
```
       #Get the path to the package share directory
        package_share_directory = get_package_share_directory('tomatoes_detection1')
        
        # Construct the path to the YOLO model weights
        self.weights_path = os.path.join(package_share_directory, 'weights/best.pt')
        
        # Load the YOLO model
        self.model = YOLO(self.weights_path)
```
In the setup.py we had to edit the data_files, by including the weights directory:
```
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the weights directory
        (f'share/{package_name}/weights', ['weights/best.pt']),
```

7. If all is setup and everything is in the right path, build the workspace again using colcon build and source ~/ros2_ws/install/setup.bash after. Then run the code " ros2 run tomatoes_detection1 task2_solution" 

8. Robot would start detecting the tomatoes when camera data is available. 

Command to run the solution:

```
source ~/ros2_ws/install/setup.bash
ros2 run tomatoes_detection1 task2_solution
```

## Challenges Faced
(Task 2)
- Detecting tomatoes was challenging due to their tendency to cluster together, complicating image processing. Green tomatoes occasionally confused the model, 
 causing it to misidentify them as weeds.--------------------------------------------------
- Internet connectivity issues.
- Insufficient GPU for image processing tasks.
- Difficulty in distinguishing closely packed tomatoes, resembling grape clusters.
- Misidentification of green tomatoes as weeds.





