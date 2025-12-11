# oppy-rover




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




