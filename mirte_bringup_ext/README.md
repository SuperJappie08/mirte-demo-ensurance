# Setup instructions

Replace the following:
- `mirte_bringup/launch/minimal_master.launch.py` astra launch with this one.
- `mirte_description/mirte_master_description/urdf/orbbec_astra_pro_plus.xacro` (or change link)
- Add `goal_tolerance: 0.025` to `mirte_control/mirte_master_arm_controller/bringup/config/mitre_master_arm_control.yaml` (on gripper_controller)
- use patched version of depth camera driver (should be merged)
- Patched SRDF for moveit if necessary
- Tune PID (A tune is provided and should be loaded by running `ros2 param load /pid_wheels_controller /PATH/TO/mirte_bringup_ext/share/config/pid_params_jap.yaml`)