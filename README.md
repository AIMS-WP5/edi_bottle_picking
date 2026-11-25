# EDI Bottle Picking

Includes 3 scenarios for controlling the UR5e robot:
- `conveyor_feeding`: the main demo for AIMS 5.0 project, with full control loop from picking a bottle to inserting it into a socket.
- `constant_pose`: same as `conveyor_feeding`, except the bottle is picked from a known constant pose.
- `grasping_test`: used to test bottle picking/grasping separately.

Robot control is done through `manipulator_interface` from [edi_robot_control](https://gitlab.edi.lv/edi_robotics/robot_control/edi_robot_control).
`conveyor_feeding` and `constant_pose` also need the differential physics model to be running (https://gitlab.edi.lv/edi_robotics/robot_control/edi_ai_moveit_servo).

Configure the scenarios without re-compiling by editing the files under config/

