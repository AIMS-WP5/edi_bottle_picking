#ifndef CONSTANT_POSE_UTILS_H_
#define CONSTANT_POSE_UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/empty.hpp>
#include <chrono>


namespace constant_pose_utils
{
	class ConstantPoseUtils
    {
    public:
    ConstantPoseUtils(manipulator_interface::ManipulatorInterface& manipulator,  bool pose_from_topic, std::string pose_topic_name, bool debug = true); // Constructor

    ~ConstantPoseUtils(); // Destructor

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

    geometry_msgs::msg::Pose get_curr_grasp_pose();
    void grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool pickup();

    bool switch_controllers(std::string start_ctrl_name, std::string stop_ctrl_name);

	private:
        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_, use_pose_from_topic_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        geometry_msgs::msg::Pose curr_grasp_pose_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_dp_exec_start_;
	};

}

#endif /* CONSTANT_POSE_UTILS_H_ */