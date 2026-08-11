#ifndef CONSTANT_POSE_UTILS_H_
#define CONSTANT_POSE_UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
#include <edi_bottle_picking/scenario_poses.h>
#include <edi_bottle_picking/vacuum_commander.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <memory>


namespace constant_pose_utils
{
	class ConstantPoseUtils
    {
    public:
    ConstantPoseUtils(manipulator_interface::ManipulatorInterface& manipulator,  bool pose_from_topic, std::string pose_topic_name,
                      std::string default_controller = "joint_trajectory_controller",
                      bool debug = true, edi_bottle_picking::BackendFlags backend = {},
                      edi_bottle_picking::ScenarioPoses poses = {}); // Constructor

    ~ConstantPoseUtils(); // Destructor

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

    geometry_msgs::msg::Pose get_curr_grasp_pose();
    void grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool pickup();

	private:
        /** \brief Command the vacuum gripper (delegates to the shared VacuumCommander). */
        bool command_vacuum(bool grip);

        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_, use_pose_from_topic_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        std::unique_ptr<edi_bottle_picking::VacuumCommander> vacuum_;
        geometry_msgs::msg::Pose curr_grasp_pose_;
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
        /** Position controller to switch back to after the DP velocity segment.
            scaled_joint_trajectory_controller on the real robot. */
        std::string default_controller_;
        /** Named SRDF poses; see scenario_poses.h for the edi-vs-isaac cell split. */
        edi_bottle_picking::ScenarioPoses poses_;
	};

}

#endif /* CONSTANT_POSE_UTILS_H_ */