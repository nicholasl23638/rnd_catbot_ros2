# rnd_catbot_ros2
Adapted [Hyperdog Ros2](https://github.com/NDHANA94/hyperdog_ros2/tree/foxy) code for ros2 humble.

## Contains
This repository contains ros2 packages for quadruped robot HyperDog.
packages are :
  1. **`hyperdog_msgs`** : this package contains the msgs those used by other packages.
  
        1. `JoyCtrlCmds` : this contains the control variables of the robot from the gamepad
              - `bool[3] states` : { start, walk, side_move_mode} 
              - `uint8 gait_type` : to change the gait type
              - `geometry_msgs/Pose pose` : to control slant(x,y) and roll,pitch,yaw
              - `geometry_msgs/Vector3 gait_step` : gait_step.x = steplen_x, gait_step.y = steplen_y, gait_step.z = swing_height
              
        2. `Geometry`: this contains the parameters for coordinate of each leg and body orientation(roll,pitch,yaw)
              - `geometry_msgs/Point32 fr` : x,y,z end effector coordinates of FR leg
              - `geometry_msgs/Point32 fl` : x,y,z end effector coordinates of FL leg
              - `geometry_msgs/Point32 br` : x,y,z end effector coordinates of BR leg
              - `geometry_msgs/Point32 bl` : x,y,z end effector coordinates of BL leg
              - `geometry_msgs/Quaternion euler_ang` : roll, pitch, yaw angles
              
  2. **`hyperdog_teleop`** : this pkg creates `/hyperdog_teleop_gamepad_node. 
        - Node 1 : `/joy_node`
            This node creates commands to robot from Gamepad commands
            - subscriber : `/joy_node` 
            - publisher : `/hyperdog_joy_ctrl_cmd` using the interface `hyperdog_msgs/msg/JoyCtrlCmd`

  3. **`hyperdog_ctrl`** : This pkg has `Body_motion_planner` and `gait_generater` and creates the nodes `/command_manager_node` and `/IK_node`
        - `Body_motion_planner` : plans body motions from control commands receive from `/command_manager_node`
        - `gait_generator` : generates gaits acording to the given gait_type command from the Gamepad 
   
        - Node 1 : `/command_manager_node` 
            - subscriber : `/hyperdog_joy_ctrl_cmd` via `hyperdog_msgs/msg/JoyCtrlCmds` interface
            - publisher : `/hyperdog_geometry` via `hyperdog_msgs/msg/Geometry` interface

        - Node 2 : `/IK_node`
            - subscriber : `/hyperdog_geometry` via `hyperdog_msgs/msg/Geometry` interface
            - publisher  : `/hyperdog_jointController/commands`
  
  4. **`hyperdog_launch`** : This contains the launch file for all the above nodes and `micro_ros_agent`
  
  5. **`hyperdog_gazebo_sim`** : Gazebo simmulation 
  
  6. **`hyperdog_gazebo_joint_cmd`** : this pkg contains the node `/hyperdog_gazebo_joint_cmd` to send joint angles to gazebo
        - Node : `/hyperdog_gazebo_joint_cmd`
            - subscriber : `/hyperdog_jointController/commands`
            - publisher : '/gazebo_joint_controller/commands`


## Building

 1. Create a ROS2 workspace and build this package for ROS2 humble
 ```
 # make the workspace
 mkdir rnd_catbot_ros2/src 
 cd rnd_catbot_ros2/src
  
 # build the pkg
 cd .. 
 colcon build
 ```
 
 2. edit line 41 in hyperdog_ros2/src/hyperdog_launch/launch/hyperdog.launch.py script to configure your serial port.
 3. go to `rnd_catbot_ros2` directory and build all the packages again
 
 
 # Launching
 source the workspace  
 ```
 source rnd_catbot_ros2/install/setup.bash
 ```
 to add workspace source permenently to .bashrc:
  ```
    
    # add source 
    echo "source /home/$USER/rnd_catbot_ros2/install/setup.bash" >> ~/.bashrc
  ```
  
  to launch run following 
  ```
  ros2 launch hyperdog_launch hyperdog.launch.py
 
  ```
  
  to launch gazebo with hyperdog
  ```
  ros2 launch hyperdog_gazebo_sim hyperdog_gazebo_sim.launch.py
  ```

# Known bugs:
- gazebo-ros2-control pkg doesn't work properly. So the robot slides on the ground.
