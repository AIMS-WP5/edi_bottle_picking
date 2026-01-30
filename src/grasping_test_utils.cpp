#include <edi_bottle_picking/grasping_test_utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace grasping_test_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("grasping_test_utils");

GraspingTestUtils::GraspingTestUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, bool debug)
    : manipulator_(manipulator), debug_(debug)
{
	sub_grasp_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		grasp_pose_topic, 10, std::bind(&GraspingTestUtils::grasp_pose_callback, this, _1)
	);
}

GraspingTestUtils::~GraspingTestUtils()
{
    // Destructor implementation
}

geometry_msgs::msg::Pose GraspingTestUtils::get_next_published_pose(std::string topic_name, bool stamped_topic, int timeout_sec)
{
    auto node = rclcpp::Node::make_shared("tmp_sub_node");
    geometry_msgs::msg::Pose received_msg;
    bool received = false;

    auto callback_pose_stamped = [&received_msg, &received](geometry_msgs::msg::PoseStamped msg) {
        received_msg = msg.pose;
        received = true;
    };
    auto sub_pose_stamped = node->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name, 10, callback_pose_stamped);

    auto callback_pose = [&received_msg, &received](geometry_msgs::msg::Pose msg) {
        received_msg = msg;
        received = true;
    };
    auto sub_pose = node->create_subscription<geometry_msgs::msg::Pose>(topic_name, 10, callback_pose);

    if (stamped_topic) {
        sub_pose.reset();
    }
    else {
        sub_pose_stamped.reset();
    }

    auto start = std::chrono::steady_clock::now();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    std::chrono::milliseconds timeout = timeout_sec * 1000ms;
    while (rclcpp::ok() && !received) {
        exec.spin_some();
        if (std::chrono::steady_clock::now() - start > timeout) {
            break;
        }
        std::this_thread::sleep_for(50ms);
    }

	if (!received) {
		RCLCPP_ERROR(LOGGER, "Grasp pose not received on topic %s", topic_name.c_str());
	}
	
	return received_msg;
}

geometry_msgs::msg::Pose GraspingTestUtils::check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg) {
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

bool GraspingTestUtils::pick_up()
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
	success_ = manipulator_.activate_vacuum_gripper(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction disabled!");
	}

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to get object grasp pose");
	}
	// geometry_msgs::msg::Pose grasp_pose = GraspingTestUtils::get_next_published_pose("/grasp_pose", false, 10);
	geometry_msgs::msg::Pose grasp_pose = GraspingTestUtils::get_curr_grasp_pose();
	if (grasp_pose.position.z == 0.0) {
		RCLCPP_ERROR(LOGGER, "Grasp pose probably incorrect!");
		return 0;
	}
	geometry_msgs::msg::PoseStamped grasp_pose_stamped;
	grasp_pose_stamped.pose = grasp_pose;
	grasp_pose_stamped.header.frame_id = "realsense_455_on_stand";
	// geometry_msgs::msg::Pose pick_pose = manipulator_.transform_pose("world", "realsense_455_on_stand", grasp_pose);
	geometry_msgs::msg::PoseStamped pick_pose_stamped = manipulator_.transform_pose("world", grasp_pose_stamped);
	geometry_msgs::msg::Pose pick_pose = pick_pose_stamped.pose;
	manipulator_.world_marker->publishAxisLabeled(pick_pose, "Object_pose");
	manipulator_.world_marker->trigger();
	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to check pose angle");
	}
	pick_pose = check_pose_angle(pick_pose, 0);
	// manipulator_.world_marker->deleteAllMarkers();
	manipulator_.world_marker->publishAxisLabeled(pick_pose, "Corrected_object_pose");
	manipulator_.world_marker->trigger();

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to add collision object");
	}
	moveit_msgs::msg::CollisionObject coll_obj;
	success_ = manipulator_.add_collision_object_simple(pick_pose, "world", coll_obj);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}
	
	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to create pick moves for the object");
	}
	std::vector<geometry_msgs::msg::Pose> pick_poses = manipulator_.create_pick_moves_simple(pick_pose);

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
		manipulator_.world_marker->prompt("press 'Next' to grasp object");
	}
	success_ = manipulator_.activate_vacuum_gripper(true);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction enabled!");
	}

	success_ = manipulator_.cartesian_goal(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to move back to position after grasping");
	}
	success_ = manipulator_.predefined_pose("wait_slam");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	return true;
}

bool GraspingTestUtils::put_down()
{
	if(debug_){
		manipulator_.world_marker->prompt("press 'Next' to drop off grasped bottle");
	}
	success_ = manipulator_.predefined_pose("bottle_dropoff");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Putting back failed!");
		return 0;
	}
	success_ = manipulator_.activate_vacuum_gripper(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Putting back failed!");
	} else {
		RCLCPP_INFO(LOGGER, "Releasing successful.");
	}

	return true;
}

geometry_msgs::msg::Pose GraspingTestUtils::get_curr_grasp_pose()
{
	return curr_grasp_pose_;
}

void GraspingTestUtils::grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	curr_grasp_pose_ = msg->pose;
}

} // namespace grasping_test_utils
