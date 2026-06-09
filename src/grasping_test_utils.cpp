#include <edi_bottle_picking/grasping_test_utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace grasping_test_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("grasping_test_utils");

GraspingTestUtils::GraspingTestUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, bool debug, bool simulation, bool run_dp_switchover)
    : manipulator_(manipulator), debug_(debug), simulation_(simulation), run_dp_switchover_(run_dp_switchover)
{
	sub_grasp_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		grasp_pose_topic, 10, std::bind(&GraspingTestUtils::grasp_pose_callback, this, _1)
	);
	isaac_vacuum_client_ = manipulator.node_->create_client<std_srvs::srv::SetBool>("/vacuum_gripper/command");
	// simulation_ (== use_sim_time) signals Isaac Sim, where the control switch must also
	// flip the joint-drive gains; on real/URSim the gain-flip is a best-effort no-op.
	control_switcher_ = std::make_unique<edi_bottle_picking::ControlModeSwitcher>(
		manipulator.node_, simulation_, "joint_trajectory_controller");
}

GraspingTestUtils::~GraspingTestUtils()
{
    // Destructor implementation
}

bool GraspingTestUtils::command_vacuum(bool grip)
{
	bool ok;
	if (simulation_) {
		// Isaac/TopicBasedSystem has no UR /set_io service, so activate_vacuum_gripper()
		// would block ~5 s on it and report the grasp failed (aborting the pick). In sim
		// the suction is modelled entirely by the Isaac bridge, so skip the UR IO path.
		ok = true;
	} else {
		// Drive the real UR vacuum gripper (via UR IO) -- unchanged behaviour on hardware.
		ok = manipulator_.activate_vacuum_gripper(grip);
	}
	// Mirror the same command to Isaac Sim's (hacky) vacuum bridge. Best-effort, so a
	// real-hardware run with no bridge just logs a warning and continues.
	set_isaac_vacuum(grip);
	return ok;
}

void GraspingTestUtils::set_isaac_vacuum(bool grip)
{
	if (!isaac_vacuum_client_->service_is_ready()) {
		RCLCPP_WARN(LOGGER, "Isaac vacuum service '/vacuum_gripper/command' not available; "
		                    "skipping sim %s command", grip ? "GRIP" : "RELEASE");
		return;
	}
	auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
	request->data = grip;
	// Fire-and-forget; the response is handled on the spinning executor thread.
	isaac_vacuum_client_->async_send_request(
		request,
		[grip](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
			auto response = future.get();
			RCLCPP_INFO(LOGGER, "Isaac vacuum %s -> success=%d (%s)",
			            grip ? "GRIP" : "RELEASE", response->success, response->message.c_str());
		});
}

bool GraspingTestUtils::add_box() {
	moveit_msgs::msg::CollisionObject coll_obj;
	geometry_msgs::msg::Pose pose;
	pose.position.x = -0.45;
	pose.position.y = -0.50;
	pose.position.z = 0.93;
	bool success = manipulator_.add_collision_box(pose, "world", coll_obj, 0.40, 0.30, 0.15, 0.015);
	return success;
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
        manipulator_.world_marker_->prompt("press 'Next' to go to position above pickup place");
    }
    success_ = manipulator_.predefined_pose("wait_slam");
    if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to open the gripper");
	}
	success_ = command_vacuum(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction disabled!");
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to get object grasp pose");
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
	manipulator_.world_marker_->publishAxisLabeled(pick_pose, "Object_pose");
	manipulator_.world_marker_->trigger();
	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to check pose angle");
	}
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

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to go above box");
	}
	success_ = manipulator_.predefined_pose("above_box_1");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = manipulator_.cartesian_goal(pick_poses[0]);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = manipulator_.cartesian_goal(pick_poses[1], 15);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	manipulator_.attach_collision_object(coll_obj);
	success_ = command_vacuum(true);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction enabled!");
	}

	std::this_thread::sleep_for(100ms);
	success_ = get_grasped_status();
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Bottle not grasped!");
		command_vacuum(false);
		manipulator_.world_marker_->prompt("press 'Next' to move back above box");
		success_ = manipulator_.predefined_pose("above_box_1");
		return 0;
	}

	success_ = manipulator_.cartesian_goal(pick_poses[0], 15);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to move back above box");
	}
	success_ = manipulator_.predefined_pose("above_box_1");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to move to ai start");
	}
	success_ = manipulator_.predefined_pose("ai_start2");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	// Hand off from MoveIt trajectory control to the real-time DP (PyTorch) velocity
	// controller, run the model, then hand back. run_dp_segment() blocks until the DP
	// node signals /dp_exec_done (or times out), so control is genuinely back in position
	// mode before we continue. Gated so model-less runs can skip the switchover entirely.
	if (run_dp_switchover_) {
		if(debug_){
			manipulator_.world_marker_->prompt("press 'Next' to run the DP velocity segment");
		}
		auto dp_result = control_switcher_->run_dp_segment();
		if (!dp_result.has_value()) {
			RCLCPP_WARN(LOGGER, "DP segment timed out; restored position control");
		} else if (*dp_result) {
			RCLCPP_INFO(LOGGER, "DP segment completed successfully");
		} else {
			RCLCPP_WARN(LOGGER, "DP segment reported failure (collision/limits exceeded)");
		}
	} else {
		RCLCPP_INFO(LOGGER, "run_dp_switchover disabled: skipping controller switchover and DP start signal");
	}

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to move back to starting position");
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
		manipulator_.world_marker_->prompt("press 'Next' to drop off grasped bottle");
	}
	success_ = manipulator_.predefined_pose("inter_floor_4");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Putting back failed!");
		return 0;
	}
	success_ = command_vacuum(false);
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

bool GraspingTestUtils::get_grasped_status(int timeout_sec)
{
	if (simulation_) {
		// No UR digital IO in Isaac Sim; the (hacky) vacuum bridge always "grasps".
		RCLCPP_INFO(LOGGER, "Simulation mode: assuming successful grasp (skipping UR IO pin-17 check)");
		return true;
	}

	auto node = rclcpp::Node::make_shared("tmp_grasp_status_node");
	ur_msgs::msg::IOStates received_msg;
	bool received = false;
    auto callback = [&received_msg, &received](ur_msgs::msg::IOStates msg) {
        received_msg = msg;
		received = true;
    };
    auto sub = node->create_subscription<ur_msgs::msg::IOStates>("/io_and_status_controller/io_states", 10, callback);

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
		RCLCPP_ERROR(LOGGER, "IO states msg not received!");
		return false;
	}

	std::vector<ur_msgs::msg::Digital> digital_in_states = received_msg.digital_in_states;
	bool pin17_state = false;
	for (ur_msgs::msg::Digital digital : digital_in_states) {
		if (digital.pin == 17) {
			pin17_state = digital.state;
			RCLCPP_INFO(LOGGER, "Got pin 17 state (gripper H2 value): %d", pin17_state);
			break;
		}
	}
	return pin17_state;
}

} // namespace grasping_test_utils
