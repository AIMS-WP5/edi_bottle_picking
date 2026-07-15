#include <edi_bottle_picking/conveyor_feeding_utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace conveyor_feeding_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("conveyor_feeding_utils");

// Vertical lift safe_retreat() uses to extract the tool from the box (walls top at z~1.08)
// before a joint-space move back to a ready pose.
static constexpr double SAFE_LIFT_M = 0.20;

// MoveIt-insertion descent shallowing: if the full-depth descent can't be planned, retry in
// 2 mm steps up to 10 mm shallower. If the robot can't descend to within 10 mm of the target,
// that's a genuine failure to address differently (not silently accept a deeper shortfall).
static constexpr double DESCENT_SHALLOW_STEP_M = 0.002;  // 2 mm increments
static constexpr double DESCENT_MAX_SHALLOW_M  = 0.010;  // up to 10 mm short of full depth

// IK branch-distance cap for the above-socket move. 2.0 rad (fixed calibrated pose: any
// larger delta is a flipped/contorted branch). Grasp-aware mode must allow the legitimate
// MIRRORED postures a flipped/rolled bottle demands (~2.3-2.6 rad on a wrist/elbow joint --
// observed 2026-07-15: every spin candidate of a flipped grasp was rejected at 2.0), so it
// uses the relaxed cap and relies on GUARD2 (collision-checked descent feasibility from the
// exact arrival config) as the true arbiter of a usable branch.
static constexpr double INSERT_MAX_SEED_DELTA             = 2.0;
static constexpr double GRASP_AWARE_INSERT_MAX_SEED_DELTA = 3.5;

// Arm joint order used for /compute_ik seeds/solutions and descent pre-validation.
static const std::vector<std::string> ARM_JOINTS = {
	"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
	"wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

// Scoped planning-pipeline pin (RAII: previous pipeline restored on every exit path).
// Precision segments that operate at near-contact -- the pick inside the bottle box, the
// insertion above the socket -- must plan via OMPL even when the task-level
// planning_pipeline is cuMotion (2026-07-14 findings):
//  - the cuMotion planner node re-solves IK even for joint targets, so it may arrive in a
//    different (descent-hostile) arm basin than the seeded-IK config the guards validated;
//  - curobo validates START states against the mirrored world with sphere buffers +
//    activation margin, so any config at near-contact (tool tip at a bottle inside the box
//    walls) hard-fails INVALID_START_STATE_WORLD_COLLISION and wedges recovery.
// cuMotion keeps the long free-space moves (wait/above-box/transfers), where those
// properties are harmless. Mirrors the hybrid planned/reactive split robo-codegen-edi
// converged on for the same robot.
struct PipelineScope {
	manipulator_interface::ManipulatorInterface &m;
	std::string prev;
	PipelineScope(manipulator_interface::ManipulatorInterface &mi, const std::string &pipeline)
	    : m(mi), prev(mi.set_planning_pipeline(pipeline)) {}
	~PipelineScope() { m.set_planning_pipeline(prev); }
};

ConveyorFeedingUtils::ConveyorFeedingUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, std::string default_controller, bool debug, bool is_isaac, int max_pick_attempts,
    std::string insertion_mode, std::string socket_pose_topic, std::array<double, 3> moveit_insert_offset, double moveit_insert_above_dz,
    std::array<double, 4> moveit_insert_orientation, bool moveit_insert_descent_collision_check,
    int moveit_insert_fallback_max_waypoints, bool moveit_insert_validate_descent,
    bool grasp_aware_insertion, std::string in_hand_pose_topic, std::array<double, 3> grasp_aware_bottle_offset)
    : manipulator_(manipulator), debug_(debug), simulation_(is_isaac), max_pick_attempts_(max_pick_attempts), default_controller_(default_controller),
      insertion_mode_(insertion_mode), socket_pose_topic_(socket_pose_topic), moveit_insert_offset_(moveit_insert_offset), moveit_insert_above_dz_(moveit_insert_above_dz),
      moveit_insert_orientation_(moveit_insert_orientation), moveit_insert_descent_collision_check_(moveit_insert_descent_collision_check),
      moveit_insert_fallback_max_waypoints_(moveit_insert_fallback_max_waypoints), moveit_insert_validate_descent_(moveit_insert_validate_descent),
      grasp_aware_insertion_(grasp_aware_insertion), in_hand_pose_topic_(in_hand_pose_topic),
      grasp_aware_bottle_offset_(grasp_aware_bottle_offset)
{
	sub_grasp_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		grasp_pose_topic, 10, std::bind(&ConveyorFeedingUtils::grasp_pose_callback, this, _1)
	);
	// Continuously latch the insertion target for the MoveIt comparison segment. Created here
	// (main thread) so run_moveit_insert_segment never creates a subscription mid-run.
	sub_socket_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		socket_pose_topic_, 10, std::bind(&ConveyorFeedingUtils::socket_pose_callback, this, _1)
	);
	// Measured bottle-in-hand transform (Isaac publishes every frame; frame_id "wrist_3_link"
	// while a bottle is bonded, "none" otherwise). Latched continuously like the socket pose.
	sub_in_hand_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		in_hand_pose_topic_, 10, std::bind(&ConveyorFeedingUtils::in_hand_pose_callback, this, _1)
	);
	control_switcher_ = std::make_unique<edi_bottle_picking::ControlModeSwitcher>(
		manipulator.node_, is_isaac, default_controller_);
	isaac_vacuum_client_ = manipulator.node_->create_client<std_srvs::srv::SetBool>("/vacuum_gripper/command");
	ik_client_ = manipulator.node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

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
	// X size 0.40 -> 0.42: widen the (sim-only) box 2 cm in X so its walls sit ~1 cm farther
	// out per side while the bottle row stays put (its pitch is fixed by bottle geometry, not
	// the box). This gives the end-most bottles enough clearance for the gripper snout to
	// descend vertically without the snout grazing a wall. Keep in sync with the Isaac scene's
	// BOX_OUTER (edi_isaacsim/simplified_ur5_scene.py).
	bool success = manipulator_.add_collision_box(pose, "world", coll_obj, 0.42, 0.30, 0.15, 0.015);
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

bool ConveyorFeedingUtils::try_pick_bottle()
{
	// Pin the pick sequence to OMPL (see PipelineScope): the approach ends at near-contact with
	// the bottle inside the box walls, where cuMotion's buffered start-state validation rejects
	// every subsequent plan (INVALID_START_STATE_WORLD_COLLISION) until the arm has left the box.
	PipelineScope pipeline_scope{manipulator_, "ompl"};

	// Clean slate. The grasped-bottle collision object (id "object", added by
	// add_collision_object_simple and attached to virtual_ee_link) is created every cycle but
	// run() never detached/removed it, so stale attached bodies accumulated in the planning
	// scene and after ~23 cycles permanently blocked the pick's Cartesian planning. Detach +
	// remove any leftover before starting, so every cycle begins clean regardless of how the
	// previous one ended (success or an early-return failure). No-op when nothing is present.
	manipulator_.detach_collision_object();
	manipulator_.remove_collision_object();

	// wait_slam is no longer re-visited at the start of every attempt; it is set once at
	// scenario startup (move_to_initial_pose, before the iteration loop). Each attempt now
	// begins its motion at above_box_1 (below), reached directly from wherever the previous
	// iteration ended (ai_after_pickup) or from the startup wait_slam on the first iteration.
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
	// Raw world-frame bottle pose (pre-verticalization): the canonical grasp-orientation
	// construction below needs the bottle's actual long-axis direction.
	geometry_msgs::msg::Pose grasp_pose_raw = pick_pose_stamped.pose;
	manipulator_.world_marker_->publishAxisLabeled(pick_pose, "Object_pose");
	manipulator_.world_marker_->trigger();
	pick_pose = check_pose_angle(pick_pose, 0);
	// Grasp-aware mode: CANONICAL grasp orientation. check_pose_angle verticalizes the
	// published bottle quaternion, so the bottle's roll about its own long axis (the
	// physically irrelevant free DOF -- bottles settle at +/-20 deg rolls) and the
	// publisher's flip content leak straight into the wrist yaw, giving each grasp a
	// different gripper-to-bottle orientation. Under the legacy seat-snap that never
	// mattered (the snap re-seated the bottle canonically); with grip-in-place the bond
	// keeps whatever relationship the pick produced, and a non-canonical one forces a
	// mirrored, physically colliding wrist at insertion. Rebuild the orientation from the
	// bottle's long-axis AZIMUTH alone: yaw = azimuth(neck) + 90 deg over the top-down
	// base Rx(180), which reproduces the known-good nominal case (unflipped, unrolled
	// bottle at azimuth -90 -> exactly Rx(180)) and is invariant to roll. Flipped bottles
	// get their 180 yaw HERE, above the box where the wrist is free -- not at insertion.
	if (grasp_aware_insertion_) {
		tf2::Quaternion qb(grasp_pose_raw.orientation.x, grasp_pose_raw.orientation.y,
		                   grasp_pose_raw.orientation.z, grasp_pose_raw.orientation.w);
		tf2::Vector3 neck = tf2::Matrix3x3(qb) * tf2::Vector3(0.0, 0.0, 1.0);
		neck.setZ(0.0);
		if (neck.length() > 0.5) {
			const double alpha = std::atan2(neck.y(), neck.x());
			tf2::Quaternion q_pick =
				tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), alpha + M_PI_2) *
				tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
			q_pick.normalize();
			pick_pose.orientation.x = q_pick.x();
			pick_pose.orientation.y = q_pick.y();
			pick_pose.orientation.z = q_pick.z();
			pick_pose.orientation.w = q_pick.w();
			RCLCPP_INFO(LOGGER, "grasp-aware pick: canonical wrist yaw from bottle-axis azimuth "
			            "%.1f deg (roll/flip content of the published pose ignored)",
			            alpha * 180.0 / M_PI);
		} else {
			RCLCPP_WARN(LOGGER, "grasp-aware pick: bottle long axis near-vertical (|horiz|=%.2f); "
			            "keeping the legacy verticalized grasp orientation", neck.length());
		}
	}
	// manipulator_.world_marker_->deleteAllMarkers();
	manipulator_.world_marker_->publishAxisLabeled(pick_pose, "Corrected_object_pose");
	manipulator_.world_marker_->trigger();

	// Build the grasped-bottle collision object for the post-grasp ATTACH, but do NOT add it
	// to the planning-scene world during the approach: the approach pose ends at its surface,
	// and cuMotion (which mirrors the world without the ACM and demands buffer clearance)
	// rejects every approach IK against it (MotionGenStatus.IK_FAIL, observed 2026-07-14).
	// MoveIt/OMPL never needed it either -- the object's only prior role was approach-path
	// protection, and it has a history of causing planning conflicts (see detach note above).
	moveit_msgs::msg::CollisionObject coll_obj;
	success_ = manipulator_.add_collision_object_simple(pick_pose, "world", coll_obj, /*apply_to_world=*/false);
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

	// Reach the approach pose via an IK-seeded joint-space move (seeded from the current
	// above_box_1 config -> natural branch) instead of a Cartesian move. cartesian_goal uses
	// jump_threshold=0, which can route the approach (a translate + re-orient-to-vertical over an
	// edge bottle) through a contorted, near-singular branch the controller then fails to track
	// (state-tolerance abort), stranding the arm wedged in/under the box. The grasp descent below
	// stays Cartesian (short, straight down from the natural approach config).
	//
	// Guard the approach IK on the ARM joints only (ignore_wrist=true): even when seeded,
	// /compute_ik occasionally returns a contorted shoulder/elbow branch (elbow flipped up,
	// shoulder_lift toward horizontal) that reaches the approach pose but can't execute the
	// straight-down grasp descent (the cycle-46 / bottle_5 thrash). The top-down grasp wrist is
	// free to rotate, so the check excludes the wrist (joints 3-5). max_seed_delta=1.0: over a
	// 50-cycle run every legitimate approach measured <= 0.70 rad while the one contorted branch
	// was 1.72 rad -- a clean gap, so 1.0 rejects it with margin and zero risk to normal picks.
	// On rejection it fails fast (arm hasn't moved) -> retry re-solves from above_box_1, instead
	// of executing the convoluted approach and then thrashing on an unplannable descent.
	auto approach_joints = compute_ik_seeded(pick_poses[0], /*max_seed_delta=*/1.0, /*ignore_wrist=*/true);
	if (!approach_joints.has_value()) {
		RCLCPP_ERROR(LOGGER, "Pick action failed! (no IK for approach pose)");
		return 0;
	}
	success_ = manipulator_.joint_goal(*approach_joints);
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

	// Bottle grasped and the arm is back above the box: a complete, successful pick.
	return true;
}

bool ConveyorFeedingUtils::move_to_initial_pose()
{
	// One-time move to the scenario's initial/home pose at startup (before the iteration loop).
	// wait_slam is no longer part of the per-iteration cycle; this is the only place it is
	// commanded as a normal move (it is also kept as the safe_retreat() recovery fallback).
	maybe_prompt("press 'Next' to go to initial pose (wait_slam)");
	return manipulator_.predefined_pose("wait_slam");
}

bool ConveyorFeedingUtils::run()
{
	// Pick a bottle, retrying on failure. A failed attempt can leave the arm down in the box;
	// safe_retreat() lifts it back out to a plannable ready pose so the retry -- and the next
	// iteration -- start clean. Without this, one failed pick wedges the arm in the box and every
	// following iteration fails at its first move (above_box_1 can't be planned out of the box).
	bool grasped = false;
	for (int attempt = 1; attempt <= max_pick_attempts_ && rclcpp::ok(); ++attempt) {
		if (try_pick_bottle()) { grasped = true; break; }
		RCLCPP_WARN(LOGGER, "Pick attempt %d/%d failed; retreating to safety and retrying",
		            attempt, max_pick_attempts_);
		safe_retreat();
	}
	if (!grasped) {
		// The pick never grasped, so this iteration runs no insertion segment and would normally
		// publish no can_update edge. Isaac's cycle counter advances ONLY on the false->true release
		// edge, so a missing edge strands it: cycle_n undershoots the iteration count by the number of
		// pre-grasp pick failures, making --auto-exit (cycle_n >= N) unreachable and hanging the sim.
		// Bracket the give-up with the same freeze->release handshake a real segment uses, so Isaac
		// ticks one cycle and logs a skipped-cycle record (segment times + target pad; sentinel
		// bottle, since nothing was picked). safe_retreat() between the edges is a real trajectory and
		// supplies the dwell Isaac needs to observe the intermediate 'false'; the short sleep
		// guarantees that dwell even if the retreat returns fast. freeze_socket()/release_socket()
		// only publish the Bool (no controller switch, no dp_exec_start), so a skip has no side
		// effects on the DP velocity node or the arm's control mode.
		control_switcher_->freeze_socket();              // segment start: can_update=false (freeze)
		safe_retreat();                                  // real motion -> natural multi-second dwell
		std::this_thread::sleep_for(750ms);              // floor the dwell so Isaac always sees 'false'
		control_switcher_->release_socket();             // segment end: can_update=true -> Isaac ticks
		RCLCPP_ERROR(LOGGER, "Pick failed after %d attempt(s); published skipped-cycle handshake; "
		             "advancing to next iteration", max_pick_attempts_);
		return false;
	}

	maybe_prompt("press 'Next' to move to after pickup pose");
	success_ = manipulator_.predefined_pose("ai_after_pickup");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		safe_retreat();
		return 0;
	}

	maybe_prompt("press 'Next' to move to ai start");
	{
		// Pin the ai_start2 handoff to OMPL (see PipelineScope). ai_start2 is a named JOINT
		// target and the segment that follows is start-configuration-sensitive: the DP policy
		// was trained from the canonical ai_start2 configuration (iteration-8 matrix: cuMotion's
		// IK-re-solved arrival put cumotion x dp placements ~13 cm off), and in moveit mode this
		// config seeds the insertion IK. OMPL executes the exact joint target.
		PipelineScope dp_handoff_scope{manipulator_, "ompl"};
		success_ = manipulator_.predefined_pose("ai_start2");
	}
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		safe_retreat();
		return 0;
	}

	// Insertion segment: either the NN velocity-control DP segment (default) or the MoveIt
	// comparison segment (above-socket move + Cartesian descent). Both return the same
	// std::optional<bool>, so the outcome handling below is shared.
	const bool moveit_mode = (insertion_mode_ == "moveit");
	maybe_prompt(moveit_mode ? "press 'Next' to run the MoveIt insertion segment"
	                         : "press 'Next' to run the DP velocity segment");
	std::optional<bool> seg_result;
	if (moveit_mode) {
		seg_result = run_moveit_insert_segment();
	} else {
		// Switch to velocity control, run the DP model, and switch back once it signals
		// /dp_exec_done (blocks until done or timeout).
		seg_result = control_switcher_->run_dp_segment();
	}
	// Record the insertion outcome and report it as the iteration result (returned at the
	// end). A timeout/no-target or collision/limits/plan failure is a real error -- log it as
	// such so it is not masked by the post-insert cleanup moves (release/back-off) succeeding.
	const char* seg_name = moveit_mode ? "MoveIt insertion segment" : "DP segment";
	bool dp_ok = false;
	if (!seg_result.has_value()) {
		RCLCPP_ERROR(LOGGER, "%s did not complete (timeout / no socket target); position control retained", seg_name);
	} else if (*seg_result) {
		RCLCPP_INFO(LOGGER, "%s completed successfully", seg_name);
		dp_ok = true;
	} else {
		RCLCPP_ERROR(LOGGER, "%s reported failure (collision/limits/plan failure)", seg_name);
	}

	// Release the inserted bottle here, at the insertion point, BEFORE the robot lifts up and
	// moves away. Previously conveyor_feeding never released after the DP segment, so the
	// bottle was only dropped at the start of the next iteration (after returning to the
	// ready poses) -- it stayed attached through ai_start2/ai_after_pickup.
	maybe_prompt("press 'Next' to release the inserted bottle");
	success_ = command_vacuum(false);
	if (!success_) {
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		safe_retreat();
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
		safe_retreat();
		return 0;
	}
	maybe_prompt("press 'Next' to move back");
	success_ = manipulator_.predefined_pose("ai_after_pickup");
	if(!success_){
		RCLCPP_ERROR(LOGGER, "Pick action failed!");
		safe_retreat();
		return 0;
	}

	// The pick + cleanup moves completed; the iteration's success is the DP insertion result,
	// so a failed/timed-out DP segment is reported (and logged) as a failed iteration.
	return dp_ok;
}

bool ConveyorFeedingUtils::safe_retreat()
{
	// Recovery may start from a near-contact config (inside the box / at the socket), where
	// cuMotion rejects the start state outright -- recover via OMPL (see PipelineScope).
	PipelineScope pipeline_scope{manipulator_, "ompl"};

	// Best-effort recovery to a safe, plannable pose after a failed pick/place. Release and
	// detach any partial grasp, LIFT STRAIGHT UP out of the box (a vertical Cartesian move -- a
	// joint-space plan from inside the box would arc the tool out through a wall and fail), then
	// return to a ready pose. Each step is best-effort: log and continue, so we get as close to
	// safe as possible even if one step fails.
	command_vacuum(false);
	manipulator_.detach_collision_object();
	manipulator_.remove_collision_object();

	if (!manipulator_.lift_z(SAFE_LIFT_M)) {
		RCLCPP_WARN(LOGGER, "safe_retreat: vertical lift failed; trying a ready pose anyway");
	}

	if (manipulator_.predefined_pose("above_box_1")) {
		return true;
	}
	RCLCPP_WARN(LOGGER, "safe_retreat: could not reach 'above_box_1'; falling back to 'wait_slam'");
	if (manipulator_.predefined_pose("wait_slam")) {
		return true;
	}
	RCLCPP_ERROR(LOGGER, "safe_retreat: FAILED to reach a safe ready pose");
	return false;
}

std::optional<std::vector<double>> ConveyorFeedingUtils::compute_ik_seeded(
	const geometry_msgs::msg::Pose& target, double max_seed_delta, bool ignore_wrist,
	const std::optional<std::vector<double>>& explicit_seed)
{
	const std::vector<std::string>& arm_joints = ARM_JOINTS;
	if (!ik_client_->wait_for_service(std::chrono::seconds(2))) {
		RCLCPP_ERROR(LOGGER, "compute_ik_seeded: /compute_ik service not available");
		return std::nullopt;
	}

	// The seed config: normally the arm's ACTUAL current config -- seeds IK (attempt 1) and is
	// the reference for both the 2*pi normalization and the branch check (how far the returned
	// solution is from where the arm is), so IK returns the branch nearest the current pose (a
	// short, natural move). An explicit_seed overrides it to steer IK into a KNOWN-GOOD basin
	// when the current config itself is in a hostile one (e.g. a wrapped-wrist arrival after a
	// cuMotion transfer): the solution is then near the explicit seed instead of the arm.
	const std::vector<double> cur = explicit_seed.has_value() ? *explicit_seed
	                                                          : manipulator_.current_joint_values();
	std::vector<double> ik_seed = cur;
	// Branch check runs when max_seed_delta > 0. Two flavours:
	//  - insert (ignore_wrist=false): fixed orientation, so ANY large delta is a bad branch; the
	//    recovery is a wrist flip, so allow a 2nd attempt.
	//  - pick approach (ignore_wrist=true): the top-down grasp wrist is free to rotate, so the
	//    check ignores the wrist (joints 3-5) and measures contortion on the ARM (joints 0-2)
	//    only; a wrist flip can't fix an arm-branch flip, so there's no in-call retry -- reject
	//    and fail clean, letting safe_retreat + the next pick attempt re-solve from a new config.
	const int max_attempts = (max_seed_delta > 0.0 && !ignore_wrist) ? 2 : 1;

	for (int attempt = 1; attempt <= max_attempts; ++attempt) {
		auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
		req->ik_request.group_name = "ur_manipulator";
		req->ik_request.ik_link_name = "virtual_ee_link";
		req->ik_request.pose_stamped.header.frame_id = "world";
		req->ik_request.pose_stamped.pose = target;
		req->ik_request.avoid_collisions = true;
		req->ik_request.robot_state.joint_state.name = arm_joints;
		req->ik_request.robot_state.joint_state.position = ik_seed;
		req->ik_request.timeout.sec = 1;

		auto future = ik_client_->async_send_request(req);
		// run() executes on the main thread while the executor spins on its own thread, so
		// blocking here is safe -- the executor delivers the response and fulfils the future.
		if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
			RCLCPP_ERROR(LOGGER, "compute_ik_seeded: /compute_ik timed out");
			return std::nullopt;
		}
		auto res = future.get();
		if (res->error_code.val != res->error_code.SUCCESS) {
			RCLCPP_ERROR(LOGGER, "compute_ik_seeded: /compute_ik failed (error code %d)", res->error_code.val);
			return std::nullopt;
		}
		std::vector<double> joints(arm_joints.size(), 0.0);
		const auto& sol = res->solution.joint_state;
		for (size_t k = 0; k < arm_joints.size(); ++k) {
			for (size_t i = 0; i < sol.name.size(); ++i) {
				if (sol.name[i] == arm_joints[k]) { joints[k] = sol.position[i]; break; }
			}
		}
		// Re-wind each joint to the 2*pi-equivalent nearest the actual current config (minimal
		// motion); /compute_ik can return a 2*pi-wrapped solution (same EE pose, huge swing).
		for (size_t k = 0; k < joints.size() && k < cur.size(); ++k) {
			while (joints[k] - cur[k] > M_PI)  joints[k] -= 2.0 * M_PI;
			while (joints[k] - cur[k] < -M_PI) joints[k] += 2.0 * M_PI;
		}

		// IK-branch guard. /compute_ik can return a different solution family (branch) than the one
		// nearest the current config: same EE pose, very different joints. Executing it routes the
		// arm the long way (convoluted plan) and lands in a contorted/near-singular pose the next
		// straight-line Cartesian move can't continue from. Detect it as a large deviation from the
		// current config (after the 2*pi normalization above, so this is a real branch difference).
		if (max_seed_delta > 0.0) {
			// insert: check all 6 joints (orientation is fixed, any big delta is a bad branch).
			// pick approach (ignore_wrist): check only the arm joints 0-2 -- the wrist is free to
			// rotate for the top-down grasp, so a big wrist delta is legitimate, but a shoulder/
			// elbow flip is the contortion that breaks the grasp descent (the cycle-46 failure).
			const size_t check_n = ignore_wrist ? 3 : joints.size();
			double maxdelta = 0.0; size_t worst = 0;
			for (size_t k = 0; k < check_n && k < cur.size(); ++k) {
				double d = std::fabs(joints[k] - cur[k]);
				if (d > maxdelta) { maxdelta = d; worst = k; }
			}
			if (maxdelta > max_seed_delta) {
				if (ignore_wrist) {
					RCLCPP_WARN(LOGGER, "IK-branch guard: REJECTED contorted pick-approach IK "
					            "(attempt %d/%d): max ARM joint delta %.2f rad on %s > %.2f; "
					            "arm=[%.2f, %.2f, %.2f]",
					            attempt, max_attempts, maxdelta, arm_joints[worst].c_str(), max_seed_delta,
					            joints[0], joints[1], joints[2]);
					// A wrist flip can't fix an arm-branch flip, so don't retry in-call: fail clean
					// and let safe_retreat + the next pick attempt re-solve from a new config.
					continue;
				}
				RCLCPP_WARN(LOGGER, "IK-branch guard: REJECTED flipped/contorted insert IK "
				            "(attempt %d/%d): max joint delta %.2f rad on %s > %.2f; "
				            "wrist=[%.2f, %.2f, %.2f] (wrist_3=%.1f deg)",
				            attempt, max_attempts, maxdelta, arm_joints[worst].c_str(), max_seed_delta,
				            joints[3], joints[4], joints[5], joints[5] * 180.0 / M_PI);
				// Re-seed the next attempt with the wrist flipped (spherical-wrist alternate
				// solution: w1+pi, -w2, w3+pi) to bias /compute_ik onto the other branch.
				ik_seed = joints;
				ik_seed[3] += M_PI;
				ik_seed[4]  = -ik_seed[4];
				ik_seed[5] += M_PI;
				continue;
			}
			// Accepted: log the margin so the threshold can be tuned from real runs.
			RCLCPP_INFO(LOGGER, "IK-branch guard: accepted (%s) max checked-joint delta %.2f rad <= %.2f%s",
			            ignore_wrist ? "arm 0-2" : "all 6", maxdelta, max_seed_delta,
			            attempt > 1 ? " [RECOVERED after re-seed]" : "");
		}

		RCLCPP_INFO(LOGGER, "compute_ik_seeded: IK -> [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] (wrist_3=%.1f deg)",
		            joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[5] * 180.0 / M_PI);
		return joints;
	}

	RCLCPP_ERROR(LOGGER, "IK-branch guard: %s stayed contorted after %d attempt(s); failing "
	            "(avoids a contorted execution / dropped bottle)",
	            ignore_wrist ? "pick-approach IK (arm branch)" : "insert IK", max_attempts);
	return std::nullopt;
}

std::optional<bool> ConveyorFeedingUtils::run_moveit_insert_segment(int socket_timeout_sec)
{
	// Comparison insertion using standard MoveIt position-controlled moves instead of the NN
	// velocity segment. Reuse the can_update_socket freeze/release handshake so the Isaac side
	// samples the target once and records placement on the release edge -- but do NOT switch
	// the controller/driver (stay in joint_trajectory_controller / position control) and never
	// touch the velocity gains. The bottle stays gripped throughout; run() does the vacuum-off
	// and detach afterwards, exactly as for the DP path.

	// Pin the WHOLE insertion segment to OMPL (see PipelineScope): the above-socket move must
	// land in the exact, descent-validated joint configuration, which cuMotion's IK-re-solving
	// joint-goal handling cannot guarantee.
	PipelineScope pipeline_scope{manipulator_, "ompl"};

	control_switcher_->freeze_socket();

	// Sample the insertion target (socket centre) from the continuously-latched member
	// subscription (Isaac publishes /socket_center every frame; can_update_socket=false has
	// frozen its advance, so the value is stable). Wait briefly for the first message; the main
	// MultiThreadedExecutor services the subscription, so a sleep-poll on the flag suffices.
	{
		auto start = std::chrono::steady_clock::now();
		while (rclcpp::ok() && !socket_received_.load() &&
		       std::chrono::steady_clock::now() - start < std::chrono::seconds(socket_timeout_sec)) {
			std::this_thread::sleep_for(50ms);
		}
	}
	if (!socket_received_.load()) {
		RCLCPP_ERROR(LOGGER, "MoveIt insertion: no socket target on '%s'; aborting segment",
		             socket_pose_topic_.c_str());
		control_switcher_->release_socket();
		return std::nullopt;
	}
	geometry_msgs::msg::Pose socket = curr_socket_pose_;

	// Use the FIXED vertical-insertion orientation (config moveit_insert_orientation_xyzw), NOT
	// the ai_start2 orientation: it holds the gripper horizontal so the bottle hangs straight
	// down (wrist_3 ~ -176 deg), which keeps the wrist clear of the table on the vertical
	// Cartesian descent. The MoveIt plan to above_pose handles the ~180 deg wrist flip; the
	// descent reuses the same orientation so the tool pose is unchanged while descending.
	geometry_msgs::msg::Pose insert_pose;
	insert_pose.orientation.x = moveit_insert_orientation_[0];
	insert_pose.orientation.y = moveit_insert_orientation_[1];
	insert_pose.orientation.z = moveit_insert_orientation_[2];
	insert_pose.orientation.w = moveit_insert_orientation_[3];
	insert_pose.position.x = socket.position.x + moveit_insert_offset_[0];
	insert_pose.position.y = socket.position.y + moveit_insert_offset_[1];
	insert_pose.position.z = socket.position.z + moveit_insert_offset_[2];

	// Candidate insert poses. Fixed mode: exactly one (the calibrated pose above). Grasp-aware
	// mode: EE targets derived from the MEASURED bottle-in-hand transform so the bottle --
	// however it actually sits in the gripper (grip-in-place) -- ends upright over the socket;
	// several world-Z spins of the (axis-symmetric) bottle are offered, best-first, because a
	// mirrored/flipped grasp makes the closest-to-calibrated spin IK-hostile while a rotated
	// one is clean (2026-07-15 test: flipped bottles failed all guards at phi*, the +/-90/180
	// candidates recover them). Falls back to the fixed pose if no valid in-hand pose exists.
	std::vector<geometry_msgs::msg::Pose> insert_candidates{insert_pose};
	if (grasp_aware_insertion_) {
		auto derived = compute_grasp_aware_insert_candidates(socket);
		if (!derived.empty()) {
			// Delta of the best candidate vs the fixed calibrated pose: ~0 with the canonical
			// seat (the live regression anchor for the derivation); large under grip-in-place.
			const auto& d = derived.front();
			const double dp = std::sqrt(
				std::pow(d.position.x - insert_pose.position.x, 2) +
				std::pow(d.position.y - insert_pose.position.y, 2) +
				std::pow(d.position.z - insert_pose.position.z, 2));
			tf2::Quaternion qd(d.orientation.x, d.orientation.y, d.orientation.z, d.orientation.w);
			tf2::Quaternion qf(insert_pose.orientation.x, insert_pose.orientation.y,
			                   insert_pose.orientation.z, insert_pose.orientation.w);
			const double dang = qd.angleShortestPath(qf) * 180.0 / M_PI;
			RCLCPP_INFO(LOGGER, "grasp-aware insertion: derived EE target pos=[%.4f, %.4f, %.4f] "
			            "orient(xyzw)=[%.4f, %.4f, %.4f, %.4f]; delta vs fixed calibrated pose: "
			            "%.1f mm, %.1f deg (expected ~0 with the canonical seat); %zu spin candidates",
			            d.position.x, d.position.y, d.position.z,
			            d.orientation.x, d.orientation.y, d.orientation.z, d.orientation.w,
			            dp * 1000.0, dang, derived.size());
			insert_candidates = derived;
		}
	}

	// Move 1: joint-space MoveIt plan to the pose above the socket. Resolve IK ourselves
	// (seeded from the current config) and plan to that explicit joint target, so the arm
	// reaches the NATURAL config -- a short move, and the one the vertical descent continues
	// from -- instead of a contorted branch a pose target might pick.
	maybe_prompt("press 'Next' to move above the socket");
	geometry_msgs::msg::Pose above_pose;
	// GUARD 2 on the SEEDED path too: a seed-accepted IK branch can still be descent-hostile --
	// e.g. a wrapped-wrist arrival (wrist_1 ~ -275 deg) whose straight-down descent would sweep
	// the wrist through the table (observed 2026-07-14 with planning_pipeline=isaac_ros_cumotion:
	// descent failed even 10 mm shallow and the bottle was released from above_dz). The seed
	// varies with each bottle's grasp yaw, so any pipeline can draw such a branch. Validate the
	// descent from the IK config BEFORE executing the move, exactly as the fallback does.
	// The collision-checked descent validation is the arbiter that keeps hostile spin
	// candidates from executing. Empirical (2026-07-15, grip-in-place): a fully mirrored
	// wrist (flipped bottle, ~135 deg from calibrated) fails it at fraction ~0.3 -- and a
	// collisions-off trial run proved that verdict PHYSICAL, not model conservatism: the
	// executed mirrored descent hit the pad (asymmetric camera-arm holder), the controller
	// aborted mid-descent, and the wedged near-contact arm state killed every following
	// cycle. Do NOT bypass this check to "rescue" mirrored candidates; flipped bottles fail
	// cleanly up front (arm safe, next cycles unaffected) until a finer attached-object
	// model can tell true holder collisions from unmodelled-socket false positives.
	const bool descent_collision_check = moveit_insert_descent_collision_check_;
	// (Captures above_pose/insert_pose by reference: validates the CURRENT candidate.)
	auto descent_ok = [&](const std::vector<double> &joints) {
		return !moveit_insert_validate_descent_ ||
		       manipulator_.cartesian_descent_feasible(ARM_JOINTS, joints,
		               above_pose, insert_pose, descent_collision_check,
		               DESCENT_SHALLOW_STEP_M, DESCENT_MAX_SHALLOW_M);
	};
	std::optional<std::vector<double>> above_joints;
	const double max_seed_delta = grasp_aware_insertion_
		? GRASP_AWARE_INSERT_MAX_SEED_DELTA : INSERT_MAX_SEED_DELTA;
	for (size_t ci = 0; ci < insert_candidates.size() && !above_joints.has_value(); ++ci) {
		insert_pose = insert_candidates[ci];
		above_pose = insert_pose;
		above_pose.position.z += moveit_insert_above_dz_;

		// Primary path: seeded /compute_ik (max_seed_delta rejects the flipped-wrist branch,
		// wrist_3 ~ +4 deg instead of -176 deg) + joint_goal to that explicit joint target.
		above_joints = compute_ik_seeded(above_pose, max_seed_delta);
		if (above_joints.has_value() && !descent_ok(*above_joints)) {
			RCLCPP_WARN(LOGGER, "MoveIt insertion: GUARD2 seeded-IK above-config is descent-hostile "
			            "(straight-down descent unplannable from it); retrying IK from the canonical "
			            "insert-ready seed");
			above_joints.reset();
		}
		// Second seeded attempt from the CANONICAL insert-ready posture: when the arm's current
		// config is itself in a hostile basin (a cuMotion transfer may land there), current-seeded
		// IK can only return that basin. Seeding from the known-descendable above-socket posture
		// (the natural branch every baseline insertion used; wrist_1 ~ -63 deg, wrist_2 ~ +52 deg)
		// steers IK back to it. The larger seed delta only bounds the solution's distance from THIS
		// seed, not from the arm.
		if (!above_joints.has_value()) {
			static const std::vector<double> NATURAL_INSERT_SEED = {
				-1.444, -1.399, 2.497, -1.097, 0.912, -3.076};
			above_joints = compute_ik_seeded(above_pose, max_seed_delta,
			                                 /*ignore_wrist=*/false, NATURAL_INSERT_SEED);
			if (above_joints.has_value() && !descent_ok(*above_joints)) {
				RCLCPP_WARN(LOGGER, "MoveIt insertion: GUARD2 natural-seed above-config also "
				            "descent-hostile; %s",
				            ci + 1 < insert_candidates.size()
				                ? "trying the next spin candidate"
				                : "trying the guarded pose_goal fallback");
				above_joints.reset();
			}
		}
		if (above_joints.has_value() && ci > 0) {
			RCLCPP_INFO(LOGGER, "grasp-aware insertion: spin candidate %zu/%zu accepted by the "
			            "IK + descent guards (earlier candidates were IK-hostile)",
			            ci + 1, insert_candidates.size());
		}
	}
	if (!above_joints.has_value()) {
		// All candidates rejected: run the global fallback against the preferred candidate.
		insert_pose = insert_candidates.front();
		above_pose = insert_pose;
		above_pose.position.z += moveit_insert_above_dz_;
	}

	manipulator_.world_marker_->publishAxisLabeled(above_pose, "Insert_above");
	manipulator_.world_marker_->publishAxisLabeled(insert_pose, "Insert_target");
	manipulator_.world_marker_->trigger();

	bool reached_above = above_joints.has_value() && manipulator_.joint_goal(*above_joints);
	if (!reached_above) {
		// Fallback: pose_goal (setPoseTarget + the global OMPL planner). /compute_ik is a local,
		// seed-sensitive solver; with base_table in the scene the local natural solutions near
		// the post-pick seed can be in collision, so it converges to the flipped branch and the
		// guard (rightly) rejects it -- the cycle-49 / -X-reach drop. A collision-free NATURAL
		// solution still exists at these poses (verified by probe_cycle49.py); the global planner
		// searches IK branches broadly and finds it, recovering the insert instead of dropping.
		// BUT the global planner can also pick a contorted/near-singular branch via a very long,
		// twisted path (iter-13: 158 waypoints, ~40 s execution, then a descent that only achieves
		// ~40%). So PLAN ONLY here and run two pre-execution guards; on either trip, fail the
		// iteration (skip this socket target) instead of executing the long move just to drop the
		// bottle high.
		RCLCPP_WARN(LOGGER, "MoveIt insertion: seeded-IK above-socket move failed (%s); "
		            "falling back to pose_goal (global plan) -- guarded",
		            above_joints.has_value() ? "joint_goal unplannable" : "no clean IK branch");

		auto pr = manipulator_.plan_pose_goal(above_pose);   // plan only; do not execute yet
		if (!pr.ok) {
			RCLCPP_ERROR(LOGGER, "MoveIt insertion: above-socket move failed "
			             "(seeded IK and pose_goal fallback both failed to plan)");
			control_switcher_->release_socket();
			return false;
		}

		// GUARD 1 -- waypoint cap: reject a clearly twisted fallback plan before executing it.
		if (pr.geometric_waypoints > moveit_insert_fallback_max_waypoints_) {
			RCLCPP_ERROR(LOGGER, "MoveIt insertion: GUARD1 fallback plan has %.0f waypoints > cap %d "
			             "(twisted/contorted path); failing iteration before the long move",
			             pr.geometric_waypoints, moveit_insert_fallback_max_waypoints_);
			control_switcher_->release_socket();
			return false;
		}

		// GUARD 2 -- descent pre-validation: confirm a straight-down descent is plannable FROM the
		// planned above-config (not the current state) before committing the move. Mirrors the real
		// descent's shallowing salvage + collision setting, so it accepts exactly what the descent
		// itself would; a near-singular arrival (iter-13) is caught here up front.
		if (moveit_insert_validate_descent_ &&
		    !manipulator_.cartesian_descent_feasible(pr.goal_joint_names, pr.goal_positions,
		            above_pose, insert_pose, descent_collision_check,
		            DESCENT_SHALLOW_STEP_M, DESCENT_MAX_SHALLOW_M)) {
			RCLCPP_ERROR(LOGGER, "MoveIt insertion: GUARD2 straight-down descent infeasible from the "
			             "planned above-config (near-singular/contorted arrival); failing iteration "
			             "before the long move");
			control_switcher_->release_socket();
			return false;
		}

		// Both guards passed -- execute the (vetted) fallback move.
		if (!manipulator_.execute_plan(pr.plan)) {
			RCLCPP_ERROR(LOGGER, "MoveIt insertion: above-socket fallback plan vetted but execution failed");
			control_switcher_->release_socket();
			return false;
		}
		RCLCPP_INFO(LOGGER, "MoveIt insertion: reached above-socket via guarded pose_goal fallback (%.0f waypoints)",
		            pr.geometric_waypoints);
	}

	// Move 2: straight-down Cartesian descent into the socket. Collision checking is
	// configurable (moveit_insert_descent_collision_check): with it on, MoveIt may abort if the
	// planning scene's solid table/pad (no socket hole modelled) blocks the descent; turn it off
	// to match the DP segment, which had no MoveIt collision checking. The descent stays a
	// fixed-orientation straight line; depth is bounded by moveit_insert_offset_xyz[z].
	//
	// Shallowing salvage: if the full-depth descent can't be planned (e.g. a collision near the
	// very bottom), retry in 2 mm steps up to 10 mm shallower -- a partial insertion beats
	// dropping the bottle. Full depth (dz=0) is tried first, so normal inserts are unaffected;
	// failing to descend to within 10 mm is a genuine failure (returned as such).
	maybe_prompt("press 'Next' to lower the bottle into the socket");
	bool descended = false;
	for (double dz = 0.0; dz <= DESCENT_MAX_SHALLOW_M + 1e-9; dz += DESCENT_SHALLOW_STEP_M) {
		geometry_msgs::msg::Pose descend_pose = insert_pose;
		descend_pose.position.z += dz;
		if (manipulator_.cartesian_goal(descend_pose, 50.0, /*avoid_collisions=*/descent_collision_check)) {
			if (dz > 0.0) {
				RCLCPP_WARN(LOGGER, "MoveIt insertion: full-depth descent unplannable; inserted "
				            "%.0f mm shallower (z+%.3f) -- partial insertion", dz * 1000.0, dz);
			}
			descended = true;
			break;
		}
	}
	if (!descended) {
		RCLCPP_ERROR(LOGGER, "MoveIt insertion: Cartesian descent failed even %.0f mm shallow; "
		            "genuine failure", DESCENT_MAX_SHALLOW_M * 1000.0);
		control_switcher_->release_socket();
		return false;
	}

	// Segment complete; the bottle is still gripped here, so the Isaac side records placement
	// accuracy on this can_update_socket release edge (before run() releases the vacuum).
	control_switcher_->release_socket();
	return true;
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

void ConveyorFeedingUtils::socket_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	curr_socket_pose_ = msg->pose;
	socket_received_.store(true);
}

void ConveyorFeedingUtils::in_hand_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	curr_in_hand_pose_ = msg->pose;
	curr_in_hand_frame_ = msg->header.frame_id;
	in_hand_received_.store(true);
}

std::vector<geometry_msgs::msg::Pose> ConveyorFeedingUtils::compute_grasp_aware_insert_candidates(
    const geometry_msgs::msg::Pose& socket)
{
	// Validity gate: Isaac publishes frame_id "none" (sentinel) whenever no bottle is bonded,
	// so a stale transform from a previous cycle can never leak into this one.
	if (!in_hand_received_.load() || curr_in_hand_frame_.empty() || curr_in_hand_frame_ == "none") {
		RCLCPP_WARN(LOGGER, "grasp-aware insertion: no valid in-hand pose on '%s' (frame_id='%s'); "
		            "falling back to the fixed calibrated insert pose",
		            in_hand_pose_topic_.c_str(),
		            curr_in_hand_frame_.empty() ? "<none received>" : curr_in_hand_frame_.c_str());
		return {};
	}

	// T_ee_bottle: the measured bottle pose re-expressed in the EE (virtual_ee_link) frame.
	// transform_pose resolves the static wrist_3_link -> virtual_ee_link chain via TF.
	geometry_msgs::msg::PoseStamped in_hand_stamped;
	in_hand_stamped.header.frame_id = curr_in_hand_frame_;
	in_hand_stamped.pose = curr_in_hand_pose_;
	geometry_msgs::msg::PoseStamped bottle_in_ee;
	try {
		bottle_in_ee = manipulator_.transform_pose("virtual_ee_link", in_hand_stamped);
	} catch (const std::exception& e) {
		RCLCPP_WARN(LOGGER, "grasp-aware insertion: TF '%s' -> 'virtual_ee_link' failed (%s); "
		            "falling back to the fixed calibrated insert pose",
		            curr_in_hand_frame_.c_str(), e.what());
		return {};
	}
	tf2::Transform T_eb;
	tf2::fromMsg(bottle_in_ee.pose, T_eb);

	// Desired BOTTLE pose: upright (local +Z up -- the asset's long axis, neck up, matching
	// every recorded successful insertion), origin at socket + grasp_aware_bottle_offset_.
	// The spin about world Z is a free DOF (the bottle is axis-symmetric); choose it so the
	// resulting EE orientation is as close as possible to the calibrated fixed orientation,
	// keeping the IK solution in the known-good basin. Closed form: with q_be the EE
	// orientation for zero spin, dot(qz(phi) * q_be, q_cal) = A cos(phi/2) + B sin(phi/2),
	// maximised at phi = 2 atan2(B, A), where A = <q_be, q_cal> and B = <k * q_be, q_cal>
	// (k the pure-z unit quaternion). |q| double-cover is absorbed by the atan2 branch.
	tf2::Quaternion q_be = T_eb.getRotation().inverse();   // EE orientation when bottle frame == world axes
	tf2::Quaternion q_cal(moveit_insert_orientation_[0], moveit_insert_orientation_[1],
	                      moveit_insert_orientation_[2], moveit_insert_orientation_[3]);
	tf2::Quaternion k(0.0, 0.0, 1.0, 0.0);                 // pure z
	tf2::Quaternion kq = k * q_be;
	const double A = q_be.x() * q_cal.x() + q_be.y() * q_cal.y() + q_be.z() * q_cal.z() + q_be.w() * q_cal.w();
	const double B = kq.x() * q_cal.x() + kq.y() * q_cal.y() + kq.z() * q_cal.z() + kq.w() * q_cal.w();
	const double phi_star = 2.0 * std::atan2(B, A);

	// Candidate spins: phi* is the closest to the calibrated orientation by construction;
	// +/-90 and 180 give the seeded-IK + descent guards reachable alternatives when phi* is
	// IK-hostile (mirrored/flipped grasps). Order the rest by their own closeness to q_cal.
	const tf2::Vector3 world_z(0.0, 0.0, 1.0);
	const tf2::Vector3 p_target(socket.position.x + grasp_aware_bottle_offset_[0],
	                            socket.position.y + grasp_aware_bottle_offset_[1],
	                            socket.position.z + grasp_aware_bottle_offset_[2]);
	std::vector<std::pair<double, double>> spins;   // (closeness score, phi)
	for (double dphi : {0.0, M_PI_2, -M_PI_2, M_PI}) {
		const double phi = phi_star + dphi;
		const double score = std::abs(A * std::cos(phi / 2.0) + B * std::sin(phi / 2.0));
		spins.emplace_back(score, phi);
	}
	std::sort(spins.begin(), spins.end(),
	          [](const auto& a, const auto& b) { return a.first > b.first; });

	std::vector<geometry_msgs::msg::Pose> candidates;
	for (const auto& [score, phi] : spins) {
		tf2::Transform T_wb;   // desired bottle pose in world
		T_wb.setRotation(tf2::Quaternion(world_z, phi));
		T_wb.setOrigin(p_target);
		// EE target: T_world_ee = T_world_bottle * inverse(T_ee_bottle).
		tf2::Transform T_we = T_wb * T_eb.inverse();
		geometry_msgs::msg::Pose ee_pose;
		tf2::toMsg(T_we, ee_pose);
		candidates.push_back(ee_pose);
	}
	return candidates;
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
