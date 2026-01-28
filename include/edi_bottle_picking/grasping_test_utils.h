#ifndef GRASPING_TEST_UTILS_H_
#define GRASPING_TEST_UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>


namespace grasping_test_utils
{
	class GraspingTestUtils
    {
    public:
    GraspingTestUtils(manipulator_interface::ManipulatorInterface& manipulator, bool debug = false); // Constructor

    ~GraspingTestUtils(); // Destructor

    bool pick_up();

    /** \brief Function to get first grasp position published by GraspGen seen by RealSense 455 camera mounted on a stand
        \param topic_name which topic to listen to
        \param stamped_topic if the topic is of type geometry_msgs::msg::PoseStamped, otherwise listens for geometry_msgs::msg::Pose
        \param timeout_sec how many seconds to wait before timeout
    */
    geometry_msgs::msg::Pose get_grasp_pose_topic(std::string topic_name, bool stamped_topic, int timeout_sec = 5);

    bool put_down();

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

	private:
        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_;
	};

}

#endif /* GRASPING_TEST_UTILS_H_ */