# My bot package

[Project based on Articulated Robotics youtube channel](https://www.youtube.com/@ArticulatedRobotics/videos)

## My bot Mk1

<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/readme_updated/imgs/my_bot_mk1_real.jpg" alt="explode"></a>
</div>
<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/readme_updated/imgs/my_bot_mk1_sim.png" alt="explode"></a>
</div>

## Simulation

### Installation for simulation:

```console
    sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-twist-mux ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-slam-toolbox ros-humble-xacro
    cd ~/your_ws/src
    git clone (this repo)
    git clone https://github.com/AdrianCobo/Pal_Gazebo_Worlds.git
    colcon build --symlink-install
    copy the gazebo models at the models folder to your gazebo models folder
```

### Simulation Launch examples:

### Gazebo Simulation and joystick control:

1. change world_name at config/map_params.yaml
2.

```console
    ros2 launch my_bot launch_sim.launch.py
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/H0Chc4LrjQw)

[![Alt text](https://img.youtube.com/vi/H0Chc4LrjQw/0.jpg)](https://www.youtube.com/watch?v=H0Chc4LrjQw)

### Gazebo Simulation and keyboard control:

1. change world_name at config/map_params.yaml
2.

```console
    ros2 launch my_bot launch_sim.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/-zVjHXezQI8)

[![Alt text](https://img.youtube.com/vi/-zVjHXezQI8/0.jpg)](https://www.youtube.com/watch?v=-zVjHXezQI8)

### Gazebo Simulation and Mapping

```console
    ros2 launch my_bot launch_sim.launch.py
    cd ~/your_ws
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/yNdHQ1RQuCk)

[![Alt text](https://img.youtube.com/vi/yNdHQ1RQuCk/0.jpg)](https://www.youtube.com/watch?v=yNdHQ1RQuCk)

### Gazebo Simulation and nav2 localization

```console
    ros2 launch my_bot launch_sim.launch.py
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    cd ~/your_ws
    ros2 launch nav2_bringup localization_launch.py map:=./src/my_bot/maps/home.yaml use_sim_time:=true
    # publish a 2D point using rviz at your robot map position
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/tctQYJnHBAQ)

[![Alt text](https://img.youtube.com/vi/tctQYJnHBAQ/0.jpg)](https://www.youtube.com/watch?v=tctQYJnHBAQ)

## Real Robot 

### Real Robot Connection Diagram

<div align="center">
<img width=500px src="https://github.com/AdrianCobo/my_bot/blob/main/circuit/Circuito.svg" alt="explode"></a>
</div>

### Real Robot Installation
    sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-twist-mux ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-slam-toolbox ros-humble-xacro
    cd ~/your_ws/src
    git clone (this repo)
    git clone https://github.com/AdrianCobo/diffdrive_arduino.git
    git clone https://github.com/AdrianCobo/diffdrive_arduino.git
    git clone https://github.com/AdrianCobo/serial
    cd diffdrive_arduino 
    git checkout 3883c00
    cd ..
    cd my_bot/description
    do the necesary changes at ros2_control.xacro at the params sectrion in order to use your own motors and your arduino device
    colcon build --symlink-install

### Real Robot controled by a joystick:

At the raspberry run:
```console
    ros2 launch my_bot launch_robot.launch.py
```

And at the pc that the joystick is connected run:
```console
    ros2 launch my_bot joystick.launch.py
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/Z-nW6ui-bXg?si=tQdZL9IHr-GUuOz4)

[![Alt text](https://img.youtube.com/vi/Z-nW6ui-bXg/0.jpg)](https://www.youtube.com/watch?v=Z-nW6ui-bXg)

### Real Robot controled by a keyboard:

At the raspberry run:
```console
    ros2 launch my_bot launch_robot.launch.py
```

And at the pc that the keyboard is connected run:
```console
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Using the Real robot Camera:

At the raspberry run:
```console
    ros2 launch my_bot camera.launch.py
```

Wherever you want to see the image run:
```console
    ros2 run rqt_image_view rqt_image_view
```

### Using the Real robot lidar:

At the raspberry run:
```console
    ros2 launch my_bot rplidar.launch.py
```

Wherever you want to see the pointClouds run:
```console
    rviz2
```

and: 
1. Change the fixed frame parameter to 'laser_frame'
2. go to: add -> by topic -> /scan


### Mapping with the real robot:

At the raspberry run:
```console
    ros2 launch my_bot launch_robot.launch.py
    ros2 launch my_bot rplidar.launch.py
```

If you want to see images from the camera go to the section: "Using the Real robot Camera"

Wherever you want to see the map construction, execute:
```console
    rviz2
    cd $(your_ws)
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=false
```

At rviz:
1. Change the fixed frame parameter to 'map'
2. go to: add -> by topic -> /map

You can see the video demonstration here: [(Youtube)](https://youtu.be/Q_-EYw8jdps?si=o-zX6n-kzWKuK6uY)

[![Alt text](https://img.youtube.com/vi/Q_-EYw8jdps/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)

### Nav2 localization with the real robot:

```console
    ros2 launch my_bot launch_sim.launch.py
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    cd ~/your_ws
    ros2 launch nav2_bringup localization_launch.py map:=./src/my_bot/maps/home.yaml use_sim_time:=true
    # publish a 2D point using rviz at your robot map position
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

You can see the video demonstration here: available soon

### Using plansys2 with the real robot:

available soon

### Real Robot Bill:

[Bill](https://github.com/AdrianCobo/my_bot/blob/readme_updated/Bill.md)

