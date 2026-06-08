#ifndef CONSTANT_POSE_UTILS_H_
#define CONSTANT_POSE_UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
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
    ConstantPoseUtils(manipulator_interface::ManipulatorInterface& manipulator,  bool pose_from_topic, std::string pose_topic_name, bool debug = true, bool is_isaac = false); // Constructor

    ~ConstantPoseUtils(); // Destructor

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

    geometry_msgs::msg::Pose get_curr_grasp_pose();
    void grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool pickup();

	private:
        /** \brief Command the vacuum gripper: in sim (is_isaac) skips the UR /set_io path
            and mirrors the command to Isaac Sim's vacuum bridge; on real hardware drives the
            UR gripper. Returns the gripper result. */
        bool command_vacuum(bool grip);
        /** \brief Best-effort SetBool call to the Isaac vacuum bridge (no-op if absent). */
        void set_isaac_vacuum(bool grip);

        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_, use_pose_from_topic_, simulation_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr isaac_vacuum_client_;
        geometry_msgs::msg::Pose curr_grasp_pose_;
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
	};

}

#endif /* CONSTANT_POSE_UTILS_H_ */