## Robot Package Template

Notes:

If you create a new file, you have to execute colcon build --symlink-install

launch:

if using gazebo:
$ ros2 launch my_bot rsp.launch.py use_sim_time:=true

gazebo + selected world:
$ ros2 launch my_bot launch_sim.launch.py world:=relative_path/obstacles.world

fast test:
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard

Rviz2:

rviz2 -d path_to_file.rviz

References: https://github.com/joshnewans/articubot_one
