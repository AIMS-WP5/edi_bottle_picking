#ifndef CONVEYOR_FEEDING_UTILS_
#define CONVEYOR_FEEDING_UTILS_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <chrono>
#include <memory>


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
        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        geometry_msgs::msg::Pose curr_grasp_pose_;
        std::string default_controller_;
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
	};

}

#endif /* CONVEYOR_FEEDING_UTILS_ */