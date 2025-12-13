# oppy-rover

## PC workspace layout (rover_ws)

On the Ubuntu Desktop 20 PC:
```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
git clone https://github.com/ToolBoxRobotics/oppy-rover.git .
cd ..
catkin_make
```

Directory tree (PC)
```text
~/rover_ws/
├── CMakeLists.txt
├── devel/
├── build/
└── src/
    ├── rover_msgs/                        # same as SBC (shared)
    ├── rover_description/                 # same as SBC (shared)
    │
    ├── rover_nav/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/
    │   │   ├── depth_hazard_monitor.py    # uses /camera/depth/points
    │   │   └── hazard_visualizer.py       # RViz markers / hazard zone overlay
    │   └── launch/
    │       └── hazard_monitor.launch
    │
    ├── rover_mission_control/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       └── mission_control.py         # Qt/PyQt GUI: mission scripts, rosbag, hazards
    │
    ├── rover_simulation/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── simulation.launch          # Gazebo + spawn rover + controllers
    │   ├── urdf/
    │   │   └── rover_gazebo.xacro         # includes rover_description/urdf/rover.xacro
    │   └── config/
    │       └── controllers.yaml
    │
    ├── moveit_opportunity_arm/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── config/
    │       ├── opportunity_arm.srdf
    │       ├── kinematics.yaml
    │       ├── controllers.yaml
    │       └── ...
    │
    ├── rover_bringup/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   ├── rover.launch               # mode:=real or mode:=sim
    │   │   ├── real_mode.launch           # includes SBC topics only
    │   │   ├── sim_mode.launch            # includes rover_simulation + nav
    │   │   ├── rviz_rover.launch
    │   │   └── moveit_demo.launch
    │   └── rviz/
    │       └── rover_full.rviz            # model + arm + hazard overlay + camera
    │
    └── rover_tools/                       # optional utilities
        ├── bag_tools.py
        ├── mission_scripts/
        └── ...
```


## Simulation Mode Usage

- Start Gazebo simulation:
```bash
roslaunch rover_bringup rover.launch mode:=sim
```

- Start MoveIt planning:
```bash
roslaunch moveit_opportunity_arm demo.launch
```

- Start interactive markers:
```bash
roslaunch rover_arm arm_interactive.launch
```

- Start RViz (auto-loaded from bringup):
```bash
roslaunch rover_bringup rviz_rover.launch
```

- Start Mission Control:
```bash
roslaunch rover_mission_control mission_control.launch
```



## Kinect / Depth Topics (Real & Sim)
- RGB:
```bash
/camera/rgb/image_color
```

- Depth image:
```bash
/camera/depth/image_raw
```

- Depth point cloud:
```bash
/camera/depth/points
```



## Logging Missions

Enable auto-logging:

Mission Control → Mission → “Auto-record during missions”

- Or manually:
```css
Start Recording
Stop Recording
```

- Files saved to:
```bash
~/rosbags/mission_YYYYMMDD_HHMMSS.bag
```




## Setup Camera Kinect in ROS Noetic
This guide will walk you through the process of setting up Kinect in ROS Noetic

### Prerequisites
- Ubuntu 20.04
```bash
lsb_release -a  #check ubuntu version
```

- ROS Noetic installed (installation instructions)
```bash
cd /opt/ros && ls #check ros distro
```
Kinect for Xbox 360 or Kinect for Windows (Kinect v1)

### Installation
- Update and upgrade
```bash
sudo apt update
sudo apt upgrade
```

- Install Dependencies
```bash
sudo apt install ros-noetic-rgbd-launch
sudo apt-get install ros-noetic-openni-launch
sudo apt-get install libfreenect-dev
```

- Install Freenect
This package is not available in the official ROS Noetic repositories, so we need to clone the GitHub repository and build it. Follow these steps to do so.
Clone the package
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/freenect_stack.git
```

Build Package 
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Run the freenect launch file
Always source the setup file
```bash
source ~/catkin_ws/devel/setup.bash
```

connect the kinect_v1 to pc
```bash
lsusb #list all conected device to pc
```

Now, we will launch the freenect example for depth registration, which allows you to obtain the point cloud with RGB data superimposed over it.
roslaunch freenect_launch freenect.launch depth_registration:=true
visualize the topics from Kinect on Rviz, open a new terminal and launch rviz.
```bash
rviz
```

Now need to setup some parameters on rviz to visualize the depth registration data.
1. In the ‘Global Options’ set the ‘Fixed Frame’ to ‘camera_link’. 
2. Add ‘pointcloud2’ object and set the topic to ‘/camera/depth_registered/points’

<img width="331" height="478" alt="image" src="https://github.com/user-attachments/assets/aeea2a97-b4ce-47c3-864e-4b61772fe0d7" />

Now wait for a few seconds to get the points on display!

#### Refrences :
- https://github.com/Shivam-Kumar-1/ros-noetic-kinectv1-setup 
- https://aibegins.net/2020/11/22/give-your-next-robot-3d-vision-kinect-v1-with-ros-noetic/
- http://www.choitek.com/uploads/5/0/8/4/50842795/ros_kinect.pdf
- http://wiki.ros.org/ROS/Tutorials/CreatingPackage
- https://naman5.wordpress.com/2014/06/24/experimenting-with-kinect-using-opencv-python-and-open-kinect-libfreenect/

