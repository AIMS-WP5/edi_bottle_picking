#ifndef GRASPING_TEST_UTILS_H_
#define GRASPING_TEST_UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <memory>


namespace grasping_test_utils
{
	class GraspingTestUtils
    {
    public:
    GraspingTestUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, bool debug = false, bool simulation = false, bool run_dp_switchover = true); // Constructor

    ~GraspingTestUtils(); // Destructor

    bool pick_up();

    /** \brief Function to get first grasp position published by selected topic
        \param topic_name which topic to listen to
        \param stamped_topic if the topic is of type geometry_msgs::msg::PoseStamped, otherwise listens for geometry_msgs::msg::Pose
        \param timeout_sec how many seconds to wait before timeout
    */
    geometry_msgs::msg::Pose get_next_published_pose(std::string topic_name, bool stamped_topic, int timeout_sec = 5);

    bool put_down();

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

    geometry_msgs::msg::Pose get_curr_grasp_pose();
    void grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool add_box();

    bool get_grasped_status(int timeout_sec = 5);

	private:
        /** \brief Command the vacuum gripper: drives the real UR gripper AND mirrors the
            command to Isaac Sim's vacuum bridge. Returns the real-gripper result. */
        bool command_vacuum(bool grip);
        /** \brief Best-effort SetBool call to the Isaac vacuum bridge (no-op if absent). */
        void set_isaac_vacuum(bool grip);

        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_;
        bool simulation_;
        bool run_dp_switchover_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr isaac_vacuum_client_;
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
        geometry_msgs::msg::Pose curr_grasp_pose_;
	};

}

#endif /* GRASPING_TEST_UTILS_H_ */