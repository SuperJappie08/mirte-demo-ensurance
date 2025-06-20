# rebet_frog
ReBeT applied a to a Turtlebot

## Prequisites:
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [Nav2](https://docs.nav2.org/getting_started/index.html#installation)

## Installation:

1. Create a workspace folder, preferrably in your home folder:
```bash
mkdir -p ~/rebet_ws/src
```

2. Clone this repository. For example:
```bash
git clone https://github.com/EGAlberts/rebet_frog.git
```
Keep in mind if you are using a specific branch or commit version that should be reflected in the clone, or remedied after cloning the default branch.

3. Clone the dependencies using VCS:
```bash
cd ~/rebet_ws
vcs import --input src/rebet_frog/frog.rosinstall src
```

3. Source your ROS2 Humble installation, install dependencies using rosdep
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```
4. To be sure, you should manually install ultralytics
```bash
pip install ultralytics
```
as well as masced_bandits
```bash
pip install masced_bandits
```

5. Build everything
```bash
colcon build --symlink-install
```

## Usage:
There are myriad launch files which need to be executed. No matter which configuration of rebet_frog you want to use the following launch files should be used:
Make sure you always have the workspace sourced when running these commands (```source ~/rebet_ws/install/setup.bash```).
### Gazebo Classic
Note: you may need to source this first for your gazebo classic to work ```bash source /usr/share/gazebo/setup.bash```.
```bash
ros2 launch rebet_frog spawn_tb3.launch.py gui:=true myseed:=1
```
For launching the simulation and spawning the turtlebot3 waffle into the world.

### Arborist
```bash
ros2 launch rebet_frog arborist_config_launch.py
```
For managing the behavior trees.

### AAL
```bash
ros2 run aal adaptation_layer
```
For providing architectural adaptations of ROS2 Nodes.

### System Reflection
```bash
ros2 run rebet_frog system_reflection.py
```
For collecting and storing system information in the BT's knowledge.

### Navigation 2
Depending on whether you plan to perform SLAM or use a pre-made map can be specified through the use_map launch argument. When use_map is false, slam_toolbox is launched to create a map.

For SLAM:
```bash
ros2 launch rebet_frog navigation_launch.py use_map:=false
```
and for the pre-made map:
```bash
ros2 launch rebet_frog navigation_launch.py use_map:=true
```

If you are choosing to perform SLAM, you can automate the process with frontier exploration:
```bash
ros2 launch wavefront_frontier frontier_launch.py 
```

### YOLO
Some of the missions expect YOLO to be running for object detection.
```bash
ros2 launch rebet_frog yolo_self_start_launch.py 
```

### Start Mission
Ultimately, to start the mission you need to use the following command:
```bash
ros2 run rebet_frog tree_action_client.py BT_NAME
```
where BT_NAME matches the name of a behavior tree defined in the [trees folder](/trees).
Right now, SLAMandCharge, FROG_BASELINE, FROG_AAL_EXTERNAL, FROG_AAL_INTERNAL are provided. The latter four referring to the evaluation of the ACSOS2024 paper about ReBeT.
Please note that these trees expect varying combinations of the above listed launch files to be used.
