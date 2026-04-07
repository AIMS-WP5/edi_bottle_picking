#include <edi_bottle_picking/constant_pose_utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace constant_pose_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("constant_pose_utils");

ConstantPoseUtils::ConstantPoseUtils(manipulator_interface::ManipulatorInterface& manipulator, bool pose_from_topic, std::string pose_topic_name, bool debug)
    : manipulator_(manipulator), debug_(debug), use_pose_from_topic_(pose_from_topic)
{
	if (use_pose_from_topic_) {
		sub_grasp_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
			pose_topic_name, 10, std::bind(&ConstantPoseUtils::grasp_pose_callback, this, _1)
		);
	}
	else {
		// initialize to a known pose
		curr_grasp_pose_.position.x = -0.429720;
		curr_grasp_pose_.position.y = 0.224214;
		curr_grasp_pose_.position.z = 0.938;
		curr_grasp_pose_.orientation.x = -0.999353691;
		curr_grasp_pose_.orientation.y = 0.012828515;
		curr_grasp_pose_.orientation.z = 0.027794998;
		curr_grasp_pose_.orientation.w = -0.00722554;
	}
}

ConstantPoseUtils::~ConstantPoseUtils()
{
    // Destructor implementation
}

void ConstantPoseUtils::grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	curr_grasp_pose_ = msg->pose;
}

geometry_msgs::msg::Pose ConstantPoseUtils::get_curr_grasp_pose()
{
	return curr_grasp_pose_;
}

geometry_msgs::msg::Pose ConstantPoseUtils::check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg) {
	// corrects grasping pose if the angle with the verical is too big

	tf2::Quaternion q(
		pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z,
		pose.orientation.w
	);
	tf2::Matrix3x3 R(q);
	tf2::Vector3 z_world(0,0,1);
	tf2::Vector3 z_gripper = R * z_world;
	tf2::Vector3 z_world_neg(0,0,-1);
	double dot_prod = z_gripper.normalized().dot(z_world_neg);
	RCLCPP_INFO(LOGGER, "Dot prod: %f", dot_prod);
	double angle_rad = std::acos(dot_prod);
	RCLCPP_INFO(LOGGER, "Angle (rad): %f", angle_rad);
	double angle_deg = angle_rad * 180.0 / M_PI;
	RCLCPP_INFO(LOGGER, "Angle (deg): %f", angle_deg);

	geometry_msgs::msg::Pose corrected_pose = pose;

	if (angle_deg > limit_deg) {
		tf2::Vector3 tilt_axis = z_world_neg.cross(z_gripper);  // perpendicular axis
		if (tilt_axis.length() < 1e-6) {
			// Edge case: perfectly aligned (or upside down)
			tilt_axis = tf2::Vector3(1,0,0);
		}
		tilt_axis.normalize();

		double delta_rotation_deg = limit_deg - angle_deg;
		double delta_rotation_rad = delta_rotation_deg * M_PI / 180.0;
		RCLCPP_INFO(LOGGER, "Delta rotation: %f deg (%f rad)", delta_rotation_deg, delta_rotation_rad);
		tf2::Quaternion rotation_quat;
		rotation_quat.setRotation(tilt_axis, delta_rotation_rad);
		tf2::Quaternion q_final = rotation_quat * q;

		q_final.normalize();

		corrected_pose.orientation.x = q_final.x();
		corrected_pose.orientation.y = q_final.y();
		corrected_pose.orientation.z = q_final.z();
		corrected_pose.orientation.w = q_final.w();
		RCLCPP_INFO(LOGGER, "Corrected pose angle by %f deg", delta_rotation_deg);
	}

	return corrected_pose;
}


bool ConstantPoseUtils::pickup()
{
    if(debug_){
        manipulator_.world_marker_->prompt("press 'Next' to go to position above pickup place");
    }
    success_ = manipulator_.test_move_manipulator_predefined("wait_slam");
    if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to get object grasp pose");
	}

	geometry_msgs::msg::Pose pick_pose = ConstantPoseUtils::get_curr_grasp_pose();
	if (pick_pose.position.z == 0.0) {
		RCLCPP_ERROR(LOGGER, "Grasp pose probably incorrect!");
		return 0;
	}
	manipulator_.world_marker_->publishAxisLabeled(pick_pose, "Object_pose");
	manipulator_.world_marker_->trigger();

	pick_pose = check_pose_angle(pick_pose, 0);
	// manipulator_.world_marker_->deleteAllMarkers();
	manipulator_.world_marker_->publishAxisLabeled(pick_pose, "Corrected_object_pose");
	manipulator_.world_marker_->trigger();

	moveit_msgs::msg::CollisionObject coll_obj;
	success_ = manipulator_.add_collision_object_simple(pick_pose, "world", coll_obj);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	std::vector<geometry_msgs::msg::Pose> pick_poses = manipulator_.create_pick_moves_simple(pick_pose);

	success_ = manipulator_.test_move_manipulator_global(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = manipulator_.test_move_manipulator_cartesian(pick_poses[1]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	manipulator_.attach_collision_object(coll_obj);

	success_ = manipulator_.activate_vacuum_gripper(true);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction enabled!");
	}

	success_ = manipulator_.test_move_manipulator_cartesian(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to move above socket");
	}
	success_ = manipulator_.test_move_manipulator_predefined("ai_start2");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
        manipulator_.world_marker_->prompt("press 'Next' to go to position above pickup place");
    }
    success_ = manipulator_.test_move_manipulator_predefined("wait_slam");
    if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}
	if(debug_){
        manipulator_.world_marker_->prompt("press 'Next' to drop off bottle");
    }

	success_ = manipulator_.test_move_manipulator_global(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = manipulator_.test_move_manipulator_cartesian(pick_poses[1]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = manipulator_.activate_vacuum_gripper(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction disabled!");
	}
	geometry_msgs::msg::Pose retreat_pose = pick_poses[1];
	retreat_pose.position.z = retreat_pose.position.z + 0.01;

	success_ = manipulator_.test_move_manipulator_cartesian(retreat_pose);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}
	success_ = manipulator_.activate_vacuum_gripper(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction disabled!");
	}

	manipulator_.detach_collision_object();

	return true;
}

} // namespace constant_pose_utils
