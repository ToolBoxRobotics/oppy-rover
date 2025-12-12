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




