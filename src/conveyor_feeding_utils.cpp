#include <edi_bottle_picking/conveyor_feeding_utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace conveyor_feeding_utils
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("conveyor_feeding_utils");

ConveyorFeedingUtils::ConveyorFeedingUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, std::string default_controller, bool debug)
    : manipulator_(manipulator), debug_(debug), default_controller_(default_controller)
{
	sub_grasp_pose_ = manipulator.node_->create_subscription<geometry_msgs::msg::PoseStamped>(
		grasp_pose_topic, 10, std::bind(&ConveyorFeedingUtils::grasp_pose_callback, this, _1)
	);
	pub_dp_exec_start_ = manipulator.node_->create_publisher<std_msgs::msg::Empty>("dp_exec_start", 10);
	sub_dp_exec_done_ = manipulator.node_->create_subscription<std_msgs::msg::Bool>(
		"dp_exec_done", 10, std::bind(&ConveyorFeedingUtils::dp_exec_done_callback, this, _1)
	);
}

ConveyorFeedingUtils::~ConveyorFeedingUtils()
{
    // Destructor implementation
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

geometry_msgs::msg::Pose ConveyorFeedingUtils::get_next_published_pose(std::string topic_name, bool stamped_topic, int timeout_sec)
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
    if(debug_){
        manipulator_.world_marker_->prompt("press 'Next' to go to position above pickup place");
    }
    success_ = manipulator_.predefined_pose("wait_slam"); //TODO: make pose nearer the box
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

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to get object grasp pose");
	}
	geometry_msgs::msg::Pose grasp_pose = ConveyorFeedingUtils::get_curr_grasp_pose();
	if (grasp_pose.position.z == 0.0) {
		RCLCPP_ERROR(LOGGER, "Grasp pose probably incorrect!");
		return 0;
	}
	geometry_msgs::msg::PoseStamped grasp_pose_stamped;
	grasp_pose_stamped.pose = grasp_pose;
	grasp_pose_stamped.header.frame_id = "realsense_455_on_stand";
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
	success_ = manipulator_.activate_vacuum_gripper(true);
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
		manipulator_.activate_vacuum_gripper(false);
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
		manipulator_.world_marker_->prompt("press 'Next' to move to after pickup pose");
	}
	success_ = manipulator_.predefined_pose("ai_after_pickup");
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

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to switch controllers");
	}
	success_ = switch_controllers("forward_velocity_controller", default_controller_);

	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to start DP control");
	}
	insertion_finished_ = false;
	auto start_msg = std_msgs::msg::Empty();
	pub_dp_exec_start_->publish(start_msg);

	while(!insertion_finished_) {
		std::this_thread::sleep_for(100ms);
	}

	RCLCPP_INFO(LOGGER, "DP execution finished.");
	success_ = switch_controllers(default_controller_, "forward_velocity_controller");

	if (insertion_successful_) {
		RCLCPP_INFO(LOGGER, "DP trajectory successful");
		manipulator_.activate_vacuum_gripper(false); // release bottle
		if(debug_){
			manipulator_.world_marker_->prompt("press 'Next' to move back");
		}
		success_ = manipulator_.predefined_pose("ai_start2");
		if(!success_){
			RCLCPP_ERROR(LOGGER, "Pick action failed!");
			return 0;
		}
		if(debug_){
			manipulator_.world_marker_->prompt("press 'Next' to move back");
		}
		success_ = manipulator_.predefined_pose("ai_after_pickup");
		if(!success_){
			RCLCPP_ERROR(LOGGER, "Pick action failed!");
			return 0;
		}

		return true;
	}

	//unsuccessful insertion
	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to move back");
	}
	manipulator_.predefined_pose("ai_start2");
	manipulator_.predefined_pose("ai_after_pickup");
	if(debug_){
		manipulator_.world_marker_->prompt("press 'Next' to drop off bottle");
	}
	manipulator_.predefined_pose("inter_floor_4");
	manipulator_.activate_vacuum_gripper(false);

	return false;
}

geometry_msgs::msg::Pose ConveyorFeedingUtils::get_curr_grasp_pose()
{
	return curr_grasp_pose_;
}

void ConveyorFeedingUtils::grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	curr_grasp_pose_ = msg->pose;
}

bool ConveyorFeedingUtils::get_grasped_status(int timeout_sec)
{
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

bool ConveyorFeedingUtils::switch_controllers(std::string start_ctrl_name, std::string stop_ctrl_name)
{
    auto node = std::make_shared<rclcpp::Node>("tmp_controller_switch_node");
    auto client = node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    // Wait for service
    while (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(LOGGER, "Waiting for /controller_manager/switch_controller service...");
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

	std::vector<std::string> start_controllers = {start_ctrl_name};
    std::vector<std::string> stop_controllers = {stop_ctrl_name};
    request->activate_controllers = start_controllers;
    request->deactivate_controllers = stop_controllers;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;

    builtin_interfaces::msg::Duration timeout;
    timeout.sec = 5;
    timeout.nanosec = 0;
    request->timeout = timeout;

    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node, future);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to call /controller_manager/switch_controller");
        return false;
    }

    // Get service response
    auto response = future.get();
    if (response->ok) {
        RCLCPP_INFO(LOGGER, "Controller switch successful");
        return true;
    }
    RCLCPP_ERROR(LOGGER, "Controller switch failed");
    return false;
}

void ConveyorFeedingUtils::dp_exec_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	insertion_finished_ = true;
	insertion_successful_ = msg->data;
}

} // namespace conveyor_feeding_utils
