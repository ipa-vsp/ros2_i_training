# Cartesian Robot Tutorial


## Introduction

This tutorial is designed to guide you through the process of building a simple Cartesian robot model using URDF (Unified Robot Description Format) and xacro within the ROS 2 Jazzy environment. The aim is to understand how to describe a robot’s physical structure and behavior in simulation using ROS tools, and how to visualize it using RViz2.

A Cartesian robot moves linearly along the X, Y, and Z axes. It is widely used in applications like 3D printing, CNC machines, and pick-and-place systems. Among other advantages, this mechanical arrangement simplifies the robot control arm solution. It has high reliability and precision when operating in three-dimensional spac.  Because of its straightforward design, it's a great choice for beginners to learn the basics of robot modeling.

This tutorial focuses on creating a robot with prismatic joint, where the link length can be customized through a xacro parameter. This teaches how to build modular, adaptable robot descriptions suitable for more advanced scenarios later.

### What You'll Learn:

- How to structure a robot model using URDF.

- How to use xacro to create reusable and configurable URDF files.

- How to launch and visualize your robot model in RViz2.

- How to parameterize robot features like link lengths for flexibility.

***Note: This tutorial is written for ROS2 Jazzy, so make sure your development environment is set up with this version***

## Task

### 1. Create Package

Create the directory: 

```bash
mkdir urdf_tutorials/src
```

then create the package
```bash 
cd urdf_tutorials/src
ros2 pkg create cartesian_robot --build-type ament_cmake 
cd cartesian_robot
```

### 2. Creating URDf file

```bash
mkdir urdf
```
Inside the `urdf` folder create the file `cartesian_urdf.xacro` which will define all the joints and links and which contains all the parameters that can be changed as required.

Copy the below urdf to the `cartesian_urdf.xacro`.

```bash
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="cartesian_robot">

  <material name="blue">
    <color rgba="0 0 1 1" />
  </material>

  <material name="red">
    <color rgba="1 0 0 1" />
  </material>

  <material name="green">
    <color rgba="0 1 0 1" />
  </material>

  <link name="base_link" />

  <xacro:property name="z_offset" value="1.0" />
  <xacro:property name="l1" value="4.0" />
  <xacro:property name="l2" value="2.4" />
  <xacro:property name="l3" value="0.8" />
  <xacro:property name="l4" value="0.2" />
  <xacro:property name="width" value="0.2" />

  <link name="link1">
    <inertial>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />          
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${l1} ${width} ${width}" />
      </geometry>
      <material name="blue" />
    </visual>
  </link>

  <link name="link1_b">
    <inertial>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />          
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${l1} ${width} ${width}" />
      </geometry>
      <material name="blue" />
    </visual>
  </link>

  <link name="link2">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1000"  ixy="0"  ixz="0" iyy="1000" iyz="0" izz="1000" />          
     </inertial>
    <visual>
      <origin xyz="0 ${l2/2} 0" rpy="0 0 0" />
      <geometry>
        <box size="${width} ${l2} ${width}" />
      </geometry>
      <material name="red" />
    </visual>
  </link>

  <link name="link3">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1000"  ixy="0"  ixz="0" iyy="1000" iyz="0" izz="1000" />          
     </inertial>
    <visual>
      <origin xyz="0 0 ${-l3/2}" rpy="0 0 0" />
      <geometry>
        <box size="${width} ${width} ${l3}" />
      </geometry>
      <material name="green" />
    </visual>
  </link>

  <link name="link4">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />          
     </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${l4} ${l4} ${l4}" />
      </geometry>
      <material name="blue" />
    </visual>
  </link> 

  <!-- Fixed link -->
  <joint name="fixed_link_1" type="fixed">
    <parent link="base_link" />
    <child link="link1" />
    <origin xyz="0 0 ${z_offset}" rpy="0 0 0" />
  </joint>

  <joint name="fixed_link_1_b" type="fixed">
    <parent link="base_link" />
    <child link="link1_b" />
    <origin xyz="0 ${l2} ${z_offset}" rpy="0 0 0" />
  </joint>

  <!-- Prismatic joint 1 -->
  <joint name="prismatic_joint_1" type="prismatic">
    <parent link="link1" />
    <child link="link2" />
    <origin xyz="${-l1/2} 0 0" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="0.0" upper="${l1}" effort="10" velocity="1.0" />
  </joint>

  <!-- Prismatic joint 2 -->
  <joint name="prismatic_joint_2" type="prismatic">
    <parent link="link2" />
    <child link="link3" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0.0" upper="${l2}" effort="100" velocity="1.0" />
  </joint>

  <!-- Prismatic joint 3 -->
  <joint name="prismatic_joint_3" type="prismatic">
    <parent link="link3" />
    <child link="link4" />
    <origin xyz="${width} 0 0" rpy="0 0 0" />
    <axis xyz="0 0 -1" />
    <limit lower="0.0" upper="${l3}" effort="100"  velocity="1.0" />
  </joint>
</robot>
```

### 3. Creating launch file

```bash
mkdir launch
```

Inside the `launch` folder create the file `cartesian_display.launch.py` and copy thhe below contents to it.

```bash
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    link_length_arg = DeclareLaunchArgument(
        name='link_length',
        default_value='1.0',
        description='Length of the robot links'
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('cartesian_robot'),
        'urdf',
        'cartesian_urdf.xacro'
    ])

    robot_description = Command([
        'xacro', ' ', urdf_path, ' link_length:=', LaunchConfiguration('link_length')
    ])

    return LaunchDescription([
        link_length_arg,

        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),       
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen'
                )
            ]
        )
    ])

```

### 4. Update `CMakeLists.txt` and `package.xml`

In `CMakeLists.txt` make sure to add this to access the urdf an dlaunch file.

```bash
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)
```

In `package.xml`, make sure you declare dependencies on:


```bash
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
```

### 5. Build the package

Return to your workspace root and build your package:

```bash
cd ~/urdf_tutorials
colcon build
source install/setup.bash
```

### 5. Visualising the robot in rviz

***Note: Before running any launch files, make sure you source your workspace***

```bash
source ~/urdf_tutorials/install/setup.bash
```

After that Run your launch file to start the robot_state_publisher and open RViz2:

```bash
ros2 launch urdf_tutorials cartesian_display.launch.py
```


![LINk](../../_source/urdf/cartesian_screen.jpg)

This will launch rviz and the joint state publisher and this allows to interact with different joints.