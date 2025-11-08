#ifndef EDI_BOTTLE_PICKING_UTILS_H_
#define EDI_BOTTLE_PICKING_UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>


namespace edi_bottle_picking_utils
{
	class EdiBottlePickingUtils
    {
    public:
    EdiBottlePickingUtils(manipulator_interface::ManipulatorInterface& manipulator, bool debug = false); // Constructor

    ~EdiBottlePickingUtils(); // Destructor

    /** \brief Function to pick up bottle
     *  - /grasp_pose topic is used to get gripper grasp pose
     *  - RealSense 435 camera mounted on the arm is used for object detection
    */
    bool pick_bottle();

    /** \brief Function to get first grasp position published on topic /grasp_pose */
    geometry_msgs::msg::Pose get_grasp_pose_topic();

    bool put_back_on_table();

	private:
        manipulator_interface::ManipulatorInterface& manipulator_;
        bool debug_, success_;
	};

}

#endif /* EDI_BOTTLE_PICKING_UTILS_H_ */