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

    /** \brief Function to get first grasp position published by GraspGen seen by RealSense 455 camera mounted on a stand */
    geometry_msgs::msg::Pose get_grasp_pose_topic();

    bool put_down();

	private:
        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_;
	};

}

#endif /* GRASPING_TEST_UTILS_H_ */