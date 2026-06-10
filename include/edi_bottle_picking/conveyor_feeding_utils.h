#ifndef CONVEYOR_FEEDING_UTILS_
#define CONVEYOR_FEEDING_UTILS_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <memory>
#include <atomic>
#include <thread>


namespace conveyor_feeding_utils
{
	class ConveyorFeedingUtils
    {
    public:
    ConveyorFeedingUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, std::string default_controller, bool debug = false, bool is_isaac = false); // Constructor

    ~ConveyorFeedingUtils(); // Destructor

    bool run();

    /** \brief Function to get first grasp position published by selected topic
        \param topic_name which topic to listen to
        \param stamped_topic if the topic is of type geometry_msgs::msg::PoseStamped, otherwise listens for geometry_msgs::msg::Pose
        \param timeout_sec how many seconds to wait before timeout
    */
    geometry_msgs::msg::Pose get_next_published_pose(std::string topic_name, bool stamped_topic, int timeout_sec = 5);

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

    geometry_msgs::msg::Pose get_curr_grasp_pose();
    void grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool add_box();

    bool get_grasped_status(int timeout_sec = 5);

	private:
        /** \brief Command the vacuum gripper: in sim (is_isaac) skips the UR /set_io path
            and mirrors the command to Isaac Sim's vacuum bridge; on real hardware drives the
            UR gripper. Returns the gripper result. */
        bool command_vacuum(bool grip);
        /** \brief Best-effort SetBool call to the Isaac vacuum bridge (no-op if absent). */
        void set_isaac_vacuum(bool grip);

        /** \brief Debug step-gate. When debug is enabled (constructor arg, live-toggled via
            /conveyor_feeding/debug), block until the user clicks 'Next' in the RViz
            RvizVisualToolsGui panel. The wait is interruptible: toggling debug off frees the
            run mid-wait with no final click. No-op when debug is disabled. */
        void maybe_prompt(const std::string& msg);

        manipulator_interface::ManipulatorInterface& manipulator_;
        std::atomic<bool> debug_;                 // live-toggled via /conveyor_feeding/debug
        std::atomic<bool> next_pressed_{false};   // set by RViz 'Next' on /rviz_visual_tools_gui
        bool success_, simulation_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr debug_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gui_sub_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr isaac_vacuum_client_;
        geometry_msgs::msg::Pose curr_grasp_pose_;
        // frame_id of the last received grasp pose; empty -> assume the camera frame.
        std::string curr_grasp_frame_;
        std::string default_controller_;
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
	};

}

#endif /* CONVEYOR_FEEDING_UTILS_ */