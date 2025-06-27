#!/bin/bash
_demo_branch="development-detection"
_info_color=$'\e[1;34m'
_warn_color=$'\e[1;33m'
_ok_color=$'\e[1;32m'
_question_color=$'\e[1;35m'
_fail_color=$'\e[1;31m'
_debug_color=$'\e[1;36m'
_reset_color=$'\e[0m'

_info="$_info_color[ INFO ]$_reset_color"
_question="$_question_color[ ???? ]$_reset_color"
_warn="$_warn_color[ WARN ]$_reset_color"
_ok="$_ok_color[  OK  ]$_reset_color"
_done="$_ok_color[ DONE ]$_reset_color"
_early_done=$'\e[1;2;31m[ DONE ]\e[0m'
_fail="$_fail_color[ FAIL ]$_reset_color"
_debug="$_debug_color[ DEBUG]$_reset_color"
_mirte=$'\e[1;3;33mMIRTE\e[0m'

echo "$_info Setting up Summer School 2025 Setup on $_mirte"

if [[ ! ("$USER" =~ ^mirte$ && "$HOSTNAME" =~ ^Mirte-.* && "$(uname -r)" =~ .*-rockchip64$ ) ]] ; then
	echo "$_fail No $_mirte detected, while this should be ran on the robot"
	read -r -p "$_question Want to continue anyway? [y/N] " response
	response=${response,,}
	if [[ ! "$response" =~ ^(y|yes)$ ]]; then
		echo "$_fail Exiting."
		exit 1
	fi
	echo "$_ok Continuing anyway..."
else
	echo "$_ok Detected $_mirte Master"
	echo "$_info Disabling ROS and Shutdown service"
	sudo service mirte-shutdown stop
	sudo service mirte-ros stop
fi

_arch=$([[ "$(uname -p)" =~ ^x86_64$ ]] && echo "amd64" || echo "arm64")

echo "$_info Adding APT repo for ros-humble-ros2-system-monitor for $_arch"

echo "deb [trusted=yes] https://github.com/ArendJan/mirte-ros-packages/raw/ros_mirte_humble_jammy_${_arch}_extra_packages/ ./" | sudo tee /etc/apt/sources.list.d/ArendJan_mirte-extra-packages.list
echo "yaml https://github.com/ArendJan/mirte-ros-packages/raw/ros_mirte_humble_jammy_${_arch}_extra_packages/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-ArendJan_mirte-extra-packages.list
echo "$_done Succesfully added apt repo for ros2-system-monitor"

echo "$_info Updating and Installing new packages"

sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-clearpath-mecanum-drive-controller ros-humble-ros2-system-monitor -y
echo "$_done Updated and Installed new packages"

if [[ ! -e /home/mirte/mirte_ws/src/mirte-ros-packages/ ]]; then
	echo "$_warn Cannot finish robot setup steps since not running on $_mirte Master"
	echo "$_early_done Finishing early"
	exit 2
else
	echo "$_info Setting up colcon-mixin"
	colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
	colcon mixin update default

	extra_path=/home/mirte/extra
	demo_path=$extra_path/mirte-demo-ensurance
	src_ws_path=/home/mirte/mirte_ws/src
	demo_ws_path=$src_ws_path/mirte-demo-ensurance
	mkdir -p $extra_path
	cd $extra_path
	git clone https://github.com/SuperJappie08/mirte-demo-ensurance.git --single-branch --branch $_demo_branch
	mkdir -p $demo_ws_path
	echo "$_info Adding mirte_bringup_ext package"
	ln -s $demo_path/mirte_bringup_ext $demo_ws_path/
	echo "$_info Adding apple_locator package"
	ln -s $demo_path/apple_locator $demo_ws_path/
	echo "$_info Installing dependencies"
	rosdep install --from-paths src --ignore-src -ry

	echo "$_info Patching robot description"
	cp $demo_path/mirte_bringup_ext/urdf/* $src_ws_path/mirte-ros-packages/mirte_description/mirte_master_description/urdf
	cp $demo_path/mirte_bringup_ext/config/mirte_master.srdf $src_ws_path/mirte-ros-packages/mirte_moveit_config/config
	cd $src_ws_path/..

	echo "$_info Patching gripper goal_tolerance"
	sudo echo "    goal_tolerance: 0.025" >> /opt/ros/humble/share/mirte_master_arm_control/config/mirte_master_arm_control.yaml
	echo "$_warn Patching PID Tune for base, this might need to be adjusted."
	sudo sed -i "s/{p: 1.0, i: 0.0, d: 0.0, i_clamp_max: 5.0, i_clamp_min: -5.0}/{p: 2.0, i: 0.5, d: 0.01, i_clamp_max: 15.0, i_clamp_min: -15.0, antiwindup: true}/g" /opt/ros/humble/share/mirte_base_control/config/mirte_base_control.yaml
	echo "$_info Patching bringup/minimal_master launch file"
	cp $demo_path/mirte_bringup_ext/launch/minimal_master.launch.py $src_ws_path/mirte-ros-packages/mirte_bringup/launch
	echo "$_done Patching finished"

	echo "$_info Building the workspace"
	colcon build --symlink-install --mixin rel-with-deb-info
	echo "$_done Finished building the workspace"

	echo "$_info Starting ROS and Shutdown service"
	sudo service mirte-ros start
	sudo service mirte-shutdown start
	echo "$_done Finished patching the $_mirte Master, Good Luck!"
fi