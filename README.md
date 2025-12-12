# oppy-rover

## SBC workspace layout (``oppy_rover_ws_sbc``)

On the Raspberry Pi CM4:
```bash
mkdir -p ~/opportunity_rover_ws_sbc/src
cd ~/opportunity_rover_ws_sbc/src
# clone your repo here
git clone https://github.com/ToolBoxRobotics/oppy-rover.git .
cd ..
catkin_make
```

### Directory tree (SBC)
```text
~/opportunity_rover_ws_sbc/
├── CMakeLists.txt          # catkin toplevel
├── devel/
├── build/
└── src/
    ├── rover_msgs/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── WheelCmd.msg
    │       ├── ArmJointCmd.msg
    │       ├── ArmJointState.msg
    │       ├── HazardParams.msg
    │       └── ...
    │
    ├── rover_description/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── urdf/
    │       ├── rover.xacro                 # main URDF/Xacro
    │       ├── rover_mesh.xacro            # GLTF/mesh-based version
    │       └── meshes/
    │           ├── mesh_0.stl              # chassis
    │           ├── mesh_1.stl              # wheel
    │           ├── mesh_2.stl              # arm link 1
    │           └── ...
    │
    ├── rover_base/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/
    │   │   └── base_bridge.py              # ROS <-> Arduino Mega (drive)
    │   ├── launch/
    │   │   ├── base_bridge.launch
    │   │   └── base_sbc_bringup.launch     # SBC side only
    │   └── firmware/
    │       └── firmware_drive.ino          # BTS7960, encoders, sensors (Mega #1)
    │
    ├── rover_arm/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/
    │   │   ├── arm_bridge.py               # ROS <-> Arduino Mega (arm steppers)
    │   │   ├── arm_homing_supervisor.py    # non-blocking homing
    │   │   └── arm_trajectory_executor.py  # track MoveIt joint trajectory
    │   ├── launch/
    │   │   ├── arm_bringup.launch
    │   │   └── arm_homing.launch
    │   └── firmware/
    │       └── firmware_arm.ino            # steppers, A4988, limit switches (Mega #2)
    │
    ├── rover_sensors/                      # (optional) SBC sensor glue
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       ├── imu_node.py                 # MPU6050 via TCA9548A
    │       ├── power_monitor_node.py       # INA219
    │       └── ...
    │
    ├── rover_bringup/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   ├── rover_sbc_real.launch       # ONLY SBC nodes (base, arm, sensors)
    │   │   └── robot_state_publisher.launch
    │   └── rviz/
    │       └── (optional minimal config)
    │
    └── (optionally) a CATKIN_IGNORE in packages that are PC-only
```
