#include <edi_bottle_picking/conveyor_feeding_utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace conveyor_feeding_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("conveyor_feeding_utils");

ConveyorFeedingUtils::ConveyorFeedingUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, std::string default_controller, bool debug, bool is_isaac)
    : manipulator_(manipulator), debug_(debug), simulation_(is_isaac), default_controller_(default_controller)
{
	sub_grasp_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		grasp_pose_topic, 10, std::bind(&ConveyorFeedingUtils::grasp_pose_callback, this, _1)
	);
	control_switcher_ = std::make_unique<edi_bottle_picking::ControlModeSwitcher>(
		manipulator.node_, is_isaac, default_controller_);
	isaac_vacuum_client_ = manipulator.node_->create_client<std_srvs::srv::SetBool>("/vacuum_gripper/command");

	// Live debug toggle: std_msgs/Bool on /conveyor_feeding/debug enables/disables the per-stage
	// 'Next' prompts while the scenario runs (consumed by maybe_prompt).
	debug_sub_ = manipulator.node_->create_subscription<std_msgs::msg::Bool>(
		"/conveyor_feeding/debug", 10,
		[this](const std_msgs::msg::Bool::SharedPtr m) {
			debug_.store(m->data);
			RCLCPP_INFO(LOGGER, "debug %s via /conveyor_feeding/debug",
			            m->data ? "ENABLED (pausing at prompts)" : "DISABLED (free-run)");
		});
	// The RViz 'Next' button publishes sensor_msgs/Joy with buttons[1]=1 on /rviz_visual_tools_gui;
	// watch it so maybe_prompt can step without the (uninterruptible) world_marker_->prompt().
	gui_sub_ = manipulator.node_->create_subscription<sensor_msgs::msg::Joy>(
		"/rviz_visual_tools_gui", 10,
		[this](const sensor_msgs::msg::Joy::SharedPtr m) {
			if (m->buttons.size() > 1 && m->buttons[1]) {
				next_pressed_.store(true);
			}
		});
}

ConveyorFeedingUtils::~ConveyorFeedingUtils()
{
    // Destructor implementation
}

bool ConveyorFeedingUtils::command_vacuum(bool grip)
{
	bool ok;
	if (simulation_) {
		// Isaac/TopicBasedSystem has no UR /set_io service, so activate_vacuum_gripper()
		// would block ~5 s on it and report the grasp failed (aborting the pick). In sim the
		// suction is modelled entirely by the Isaac bridge, so skip the UR IO path.
		ok = true;
	} else {
		ok = manipulator_.activate_vacuum_gripper(grip);
	}
	// Mirror the command to Isaac Sim's vacuum bridge (best-effort; warns + continues if absent).
	set_isaac_vacuum(grip);
	return ok;
}

void ConveyorFeedingUtils::set_isaac_vacuum(bool grip)
{
	if (!isaac_vacuum_client_->service_is_ready()) {
		RCLCPP_WARN(LOGGER, "Isaac vacuum service '/vacuum_gripper/command' not available; "
		                    "skipping sim %s command", grip ? "GRIP" : "RELEASE");
		return;
	}
	auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
	request->data = grip;
	isaac_vacuum_client_->async_send_request(
		request,
		[grip](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
			auto response = future.get();
			RCLCPP_INFO(LOGGER, "Isaac vacuum %s -> success=%d (%s)",
			            grip ? "GRIP" : "RELEASE", response->success, response->message.c_str());
		});
}

bool ConveyorFeedingUtils::add_box() {
	moveit_msgs::msg::CollisionObject coll_obj;
	geometry_msgs::msg::Pose pose;
	pose.position.x = -0.45;
	pose.position.y = -0.50;
	pose.position.z = 0.93;
	bool success = manipulator_.add_collision_box(pose, "world", coll_obj, 0.40, 0.30, 0.15, 0.015);
	return success;
}

void ConveyorFeedingUtils::maybe_prompt(const std::string& msg)
{
	// Debug step-gate. With debug on, block until the user clicks 'Next' in the RViz
	// RvizVisualToolsGui panel (buttons[1] on /rviz_visual_tools_gui). The wait is interruptible:
	// publishing false on /conveyor_feeding/debug frees the run mid-wait, so a live toggle-off
	// needs no final click. With debug off it announces the stage ("PROCEEDING TO: ...") and
	// returns, so the scenario cycles unattended but stays traceable in the log. Polls
	// wall-clock so it behaves the same regardless of use_sim_time.

	// Call sites phrase msg as "press 'Next' to <action>"; strip the lead-in so the free-run
	// log reads "PROCEEDING TO: <action>".
	static const std::string kPrefix = "press 'Next' to ";
	const std::string action = (msg.rfind(kPrefix, 0) == 0) ? msg.substr(kPrefix.size()) : msg;

	if (!debug_.load()) {
		RCLCPP_INFO(LOGGER, "PROCEEDING TO: %s", action.c_str());
		return;
	}
	RCLCPP_INFO(LOGGER, "%s  [debug] -- click 'Next' in RViz, or "
	                    "`ros2 topic pub --once /conveyor_feeding/debug std_msgs/msg/Bool \"{data: false}\"` to free-run",
	            msg.c_str());
	next_pressed_.store(false);
	while (rclcpp::ok() && debug_.load() && !next_pressed_.load()) {
		std::this_thread::sleep_for(50ms);
	}
	if (!debug_.load()) {
		// Freed by a live debug toggle-off rather than a 'Next' click -> now auto-proceeding.
		RCLCPP_INFO(LOGGER, "PROCEEDING TO: %s", action.c_str());
	}
	next_pressed_.store(false);
}

geometry_msgs::msg::Pose ConveyorFeedingUtils::get_next_published_pose(std::string topic_name, bool stamped_topic, int timeout_sec)
{
    // Inherit the parent's clock source (sim time off Isaac's /clock when use_sim_time) so
    // this short-lived node is consistent with the rest of the stack.
    rclcpp::NodeOptions sim_opts;
    sim_opts.parameter_overrides({
        rclcpp::Parameter("use_sim_time", manipulator_.node_->get_parameter("use_sim_time").as_bool())});
    auto node = rclcpp::Node::make_shared("tmp_sub_node", sim_opts);
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

geometry_msgs::msg::Pose ConveyorFeedingUtils::check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg) {
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

bool ConveyorFeedingUtils::run()
{
	// Clean slate. The grasped-bottle collision object (id "object", added by
	// add_collision_object_simple and attached to virtual_ee_link) is created every cycle but
	// run() never detached/removed it, so stale attached bodies accumulated in the planning
	// scene and after ~23 cycles permanently blocked the pick's Cartesian planning. Detach +
	// remove any leftover before starting, so every cycle begins clean regardless of how the
	// previous one ended (success or an early-return failure). No-op when nothing is present.
	manipulator_.detach_collision_object();
	manipulator_.remove_collision_object();

    maybe_prompt("press 'Next' to go to position above pickup place");
    success_ = manipulator_.predefined_pose("wait_slam"); //TODO: make pose nearer the box
    if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	success_ = command_vacuum(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction disabled!");
	}

	maybe_prompt("press 'Next' to get object grasp pose");
	geometry_msgs::msg::Pose grasp_pose = ConveyorFeedingUtils::get_curr_grasp_pose();
	if (grasp_pose.position.z == 0.0) {
		RCLCPP_ERROR(LOGGER, "Grasp pose probably incorrect!");
		return 0;
	}
	geometry_msgs::msg::PoseStamped grasp_pose_stamped;
	grasp_pose_stamped.pose = grasp_pose;
	// Honor the publisher's frame_id when set (e.g. "world" from the Isaac sim); fall
	// back to the camera frame for the real vision pipeline, which leaves it empty.
	grasp_pose_stamped.header.frame_id = curr_grasp_frame_.empty() ? "realsense_455_on_stand" : curr_grasp_frame_;
	geometry_msgs::msg::PoseStamped pick_pose_stamped = manipulator_.transform_pose("world", grasp_pose_stamped);
	geometry_msgs::msg::Pose pick_pose = pick_pose_stamped.pose;
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

	maybe_prompt("press 'Next' to go above box");
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
		maybe_prompt("press 'Next' to move back above box");
		success_ = manipulator_.predefined_pose("above_box_1");
		return 0;
	}

	success_ = manipulator_.cartesian_goal(pick_poses[0], 15);
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	maybe_prompt("press 'Next' to move back above box");
	success_ = manipulator_.predefined_pose("above_box_1");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	maybe_prompt("press 'Next' to move to after pickup pose");
	success_ = manipulator_.predefined_pose("ai_after_pickup");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	maybe_prompt("press 'Next' to move to ai start");
	success_ = manipulator_.predefined_pose("ai_start2");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	maybe_prompt("press 'Next' to run the DP velocity segment");
	// Switch to velocity control, run the DP model, and switch back once it signals
	// /dp_exec_done (blocks until done or timeout).
	auto dp_result = control_switcher_->run_dp_segment();
	// Record the DP insertion outcome and report it as the iteration result (returned at the
	// end). A timeout or collision/limits failure is a real error -- log it as such so it is
	// not masked by the post-DP cleanup moves (release/back-off) succeeding afterwards.
	bool dp_ok = false;
	if (!dp_result.has_value()) {
		RCLCPP_ERROR(LOGGER, "DP segment timed out; restored position control");
	} else if (*dp_result) {
		RCLCPP_INFO(LOGGER, "DP segment completed successfully");
		dp_ok = true;
	} else {
		RCLCPP_ERROR(LOGGER, "DP segment reported failure (collision/limits exceeded)");
	}

	// Release the inserted bottle here, at the insertion point, BEFORE the robot lifts up and
	// moves away. Previously conveyor_feeding never released after the DP segment, so the
	// bottle was only dropped at the start of the next iteration (after returning to the
	// ready poses) -- it stayed attached through ai_start2/ai_after_pickup.
	maybe_prompt("press 'Next' to release the inserted bottle");
	success_ = command_vacuum(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	} else {
		RCLCPP_INFO(LOGGER, "Suction disabled (bottle released)!");
	}

	// The bottle is now released into the socket: detach it from the gripper and remove the
	// phantom from the planning scene (mirrors manipulator_interface's place flow), so the
	// move-backs and the next pick plan against a clean scene.
	manipulator_.detach_collision_object();
	manipulator_.remove_collision_object();

	maybe_prompt("press 'Next' to move back");
	success_ = manipulator_.predefined_pose("ai_start2");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}
	maybe_prompt("press 'Next' to move back");
	success_ = manipulator_.predefined_pose("ai_after_pickup");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		return 0;
	}

	// The pick + cleanup moves completed; the iteration's success is the DP insertion result,
	// so a failed/timed-out DP segment is reported (and logged) as a failed iteration.
	return dp_ok;
}

geometry_msgs::msg::Pose ConveyorFeedingUtils::get_curr_grasp_pose()
{
	return curr_grasp_pose_;
}

void ConveyorFeedingUtils::grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	curr_grasp_pose_ = msg->pose;
	curr_grasp_frame_ = msg->header.frame_id;
}

bool ConveyorFeedingUtils::get_grasped_status(int timeout_sec)
{
	if (simulation_) {
		// No UR digital IO in Isaac Sim; the (hacky) vacuum bridge always "grasps".
		RCLCPP_INFO(LOGGER, "Simulation mode: assuming successful grasp (skipping UR IO pin-17 check)");
		return true;
	}

	// Inherit the parent's clock source for consistency with the rest of the stack.
	rclcpp::NodeOptions sim_opts;
	sim_opts.parameter_overrides({
		rclcpp::Parameter("use_sim_time", manipulator_.node_->get_parameter("use_sim_time").as_bool())});
	auto node = rclcpp::Node::make_shared("tmp_grasp_status_node", sim_opts);
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

} // namespace conveyor_feeding_utils
