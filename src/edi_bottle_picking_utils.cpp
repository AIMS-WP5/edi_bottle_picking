#include <edi_bottle_picking/edi_bottle_picking_utils.h>

using namespace std::chrono_literals;

namespace edi_bottle_picking_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("edi_bottle_picking_utils");

EdiBottlePickingUtils::EdiBottlePickingUtils(manipulator_interface::ManipulatorInterface& manipulator, bool debug)
    : manipulator_(manipulator), debug_(debug)
{
    // Constructor implementation  

}

EdiBottlePickingUtils::~EdiBottlePickingUtils()
{
    // Destructor implementation
}

geometry_msgs::msg::Pose EdiBottlePickingUtils::get_grasp_pose_topic()
{
    auto node = rclcpp::Node::make_shared("tmp_sub_node");
	geometry_msgs::msg::Pose received_msg;
	bool received = false;
    auto callback = [&received_msg, &received](geometry_msgs::msg::Pose msg) {
        received_msg = msg;
		received = true;
    };
    auto sub = node->create_subscription<geometry_msgs::msg::Pose>("/grasp_pose", 10, callback);

    auto start = std::chrono::steady_clock::now();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

	std::chrono::milliseconds timeout = 5000ms;
    while (rclcpp::ok() && !received) {
        exec.spin_some();
        if (std::chrono::steady_clock::now() - start > timeout) {
            break;
        }
        std::this_thread::sleep_for(50ms);
    }

	if (!received) {
		RCLCPP_ERROR(LOGGER, "Grasp pose not received on topic /grasp_pose");
	}
	
	return received_msg;
}

bool EdiBottlePickingUtils::pick_bottle()
{
    if(debug_){
        manipulator_.world_marker->prompt("press 'Next' to go to position above pickup place");
    }
    success_ = manipulator_.predefined_pose("wait_slam");
    if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to open the gripper");
	}	
    if(manipulator_.activate_gripper(false) != moveit::core::MoveItErrorCode::SUCCESS){
      RCLCPP_ERROR(LOGGER, "Pick action failed!");
      return 0;
	} else {
      RCLCPP_INFO(LOGGER, "Gripper opened!");
    }

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to get object grasp pose");
	}
	geometry_msgs::msg::Pose grasp_pose = EdiBottlePickingUtils::get_grasp_pose_topic();
	if (grasp_pose.position.z == 0.0) {
		RCLCPP_ERROR(LOGGER, "Grasp pose probably incorrect!");
		return 0;
	}
	geometry_msgs::msg::Pose pick_pose = manipulator_.transform_pose("world", "realsense_435_on_robot", grasp_pose);

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to add collision object");
	}
	moveit_msgs::msg::CollisionObject coll_obj;
	success_ = manipulator_.add_collision_object(pick_pose, "world", coll_obj);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}
	
	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to create pick moves for the object");
	}
	std::vector<geometry_msgs::msg::Pose> pick_poses = manipulator_.create_pick_moves(pick_pose);

	manipulator_.check_pose(pick_poses);

	success_ = manipulator_.cartesian_goal(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = manipulator_.cartesian_goal(pick_poses[1]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to attach collision object");
	}
	manipulator_.attach_collision_object(coll_obj);

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to close the gripper");
	}

	if(manipulator_.activate_gripper(true) != moveit::core::MoveItErrorCode::SUCCESS){
      RCLCPP_ERROR(LOGGER, "Pick action failed!");
      return 0;
	} else {
      RCLCPP_INFO(LOGGER, "Gripper closed!");
    }

	success_ = manipulator_.cartesian_goal(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to move std position after grasping");
	}
	success_ = manipulator_.predefined_pose("grasping_station");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	return true;
}

bool EdiBottlePickingUtils::put_back_on_table()
{
    if(debug_){
        manipulator_.world_marker->prompt("press 'Next' to place bottle back on the table");
    }
	success_ = manipulator_.predefined_pose("blue");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Putting back failed!");
        return 0;
	}
	if(manipulator_.activate_gripper(false) != moveit::core::MoveItErrorCode::SUCCESS){
      RCLCPP_ERROR(LOGGER, "Putting back failed!");
      return 0;
	} else {
      RCLCPP_INFO(LOGGER, "Gripper opened!");
    }

    return true;
}

} // namespace edi_bottle_picking_utils
