# Mirte navigation


## how to
- clone, rosdep and build
- ```ros2 launch mirte_navigation robot_navigation.launch.py```
- Creating map:
  - remove map, run normal launch file
  - ```ros2 launch slam_toolbox online_async_launch.py```
  - drive around with ```ros2 launch mirte_teleop teleop_key.launch.py```
  - after creating map: cd to maps/ and ```ros2 run nav2_map_server map_saver_cli -f map```


## Todos
- omni is not really working
- min speed is often too low
- initial pose is 'defaulted' to 0,0,0
- target pose might be too far