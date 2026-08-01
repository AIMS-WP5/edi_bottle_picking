#include <edi_bottle_picking/conveyor_feeding.h>
#include <csignal>

using namespace conveyor_feeding;
using namespace manipulator_interface;
using namespace conveyor_feeding_utils;
using namespace std::chrono_literals;

std::atomic_bool sigint_received(false);

std::string file_path = __FILE__;
auto pos = file_path.find_last_of("/");
std::string file_dir_path = file_path.substr(0, pos);
std::string config_file_path = file_dir_path + "/../config/conveyor_feeding_config.yaml";
YAML::Node config = YAML::LoadFile(config_file_path);
bool debug = config["debug"].as<bool>();
int total_iterations = config["iterations"].as<int>();
std::string grasp_pose_topic = config["grasp_pose_topic"].as<std::string>();
std::string default_controller = config["default_controller"].as<std::string>();
int max_pick_attempts = config["max_pick_attempts"] ? config["max_pick_attempts"].as<int>() : 3;
std::string insertion_mode = config["insertion_mode"] ? config["insertion_mode"].as<std::string>() : "dp";
std::string socket_pose_topic = config["socket_pose_topic"] ? config["socket_pose_topic"].as<std::string>() : "socket_center";
std::vector<double> moveit_insert_offset = config["moveit_insert_offset_xyz"]
    ? config["moveit_insert_offset_xyz"].as<std::vector<double>>() : std::vector<double>{0.0, 0.0, 0.0};
double moveit_insert_above_dz = config["moveit_insert_above_dz"] ? config["moveit_insert_above_dz"].as<double>() : 0.10;
std::vector<double> moveit_insert_orientation = config["moveit_insert_orientation_xyzw"]
    ? config["moveit_insert_orientation_xyzw"].as<std::vector<double>>() : std::vector<double>{0.515881, 0.483598, -0.515881, -0.483598};
bool moveit_insert_descent_collision_check = config["moveit_insert_descent_collision_check"]
    ? config["moveit_insert_descent_collision_check"].as<bool>() : true;
// Guards on the above-socket move's global-planner fallback (MoveIt mode only): reject an
// over-complex fallback plan, and pre-validate the descent from the planned above-config, BEFORE
// executing -- so a contorted plan fails the iteration up front instead of running a long move
// that then can't descend.
int moveit_insert_fallback_max_waypoints = config["moveit_insert_fallback_max_waypoints"]
    ? config["moveit_insert_fallback_max_waypoints"].as<int>() : 85;
bool moveit_insert_validate_descent = config["moveit_insert_validate_descent"]
    ? config["moveit_insert_validate_descent"].as<bool>() : true;
// Grasp-aware insertion (MoveIt mode only, default off): derive the insert EE pose from the
// measured bottle-in-hand transform (grasp_in_hand) instead of the fixed calibrated pose.
bool grasp_aware_insertion = config["grasp_aware_insertion"]
    ? config["grasp_aware_insertion"].as<bool>() : false;
std::string in_hand_pose_topic = config["in_hand_pose_topic"]
    ? config["in_hand_pose_topic"].as<std::string>() : "grasp_in_hand";
std::vector<double> grasp_aware_bottle_offset = config["grasp_aware_bottle_offset_xyz"]
    ? config["grasp_aware_bottle_offset_xyz"].as<std::vector<double>>() : std::vector<double>{0.0, 0.0, 0.078};
// Physical bottle / suction-tip geometry (radius-aware pick + insert). Defaults match the
// baseline bottle_v3 asset; a different-radius bottle only needs bottle_radius changed.
bool pick_depth_flush = config["pick_depth_flush"] ? config["pick_depth_flush"].as<bool>() : false;
double pick_depth_compliance = config["pick_depth_compliance"] ? config["pick_depth_compliance"].as<double>() : 0.0037;
bool moveit_insert_radius_aware = config["moveit_insert_radius_aware"] ? config["moveit_insert_radius_aware"].as<bool>() : false;
double bottle_radius = config["bottle_radius"] ? config["bottle_radius"].as<double>() : 0.0176;
double grip_offset = config["grip_offset"] ? config["grip_offset"].as<double>() : 0.012;
double suction_tip_compliance = config["suction_tip_compliance"] ? config["suction_tip_compliance"].as<double>() : 0.0037;
double seat_cup_stretch = config["seat_cup_stretch"] ? config["seat_cup_stretch"].as<double>() : 0.0011;
// Named SRDF poses: YAML defaults here, `pose_set` / per-pose ROS params applied in main()
// once the node exists. See scenario_poses.h for the edi-vs-isaac cell split.
edi_bottle_picking::ScenarioPoses scenario_poses = edi_bottle_picking::load_scenario_poses(config);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ConveyorFeeding application;

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(application.node_);

  std::thread executor_thread([&executor]() { executor.spin(); });
  executor_thread.detach();

  const rclcpp::Logger LOGGER = application.node_->get_logger();

  application.move_group_ptr = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(application.node_,"ur_manipulator"));
  
  application.gripper_action_client_ptr = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
                                            application.node_,"robotiq_gripper_controller/gripper_cmd"); //gripper

  application.tf_buffer_ptr = tfBufferPtr(new tf2_ros::Buffer(application.node_->get_clock()));
  application.tf_listener_ptr = tfListenerPtr(new tf2_ros::TransformListener(*application.tf_buffer_ptr));

  ManipulatorInterface manipulator(application.node_, application.move_group_ptr, application.gripper_action_client_ptr, 
                                   application.tf_buffer_ptr);

  // use_sim_time signals Isaac Sim (URSim/real run wall-clock); the facade only sends the
  // Isaac drive-gain flip when this is true.
  bool is_isaac = application.node_->get_parameter("use_sim_time").as_bool();

  // The YAML value is the default; a `debug` ROS param (set by conveyor_feeding.launch.py /
  // bringup_sim_stack.sh --debug|--no-debug) overrides it. It can also be toggled live at
  // runtime by publishing std_msgs/Bool on /conveyor_feeding/debug (see maybe_prompt).
  // The node auto-declares parameters from launch overrides, so the launch-provided `debug`
  // is already declared by now -- guard against re-declaring it (ParameterAlreadyDeclared).
  if (!application.node_->has_parameter("debug")) {
    application.node_->declare_parameter("debug", debug);
  }
  debug = application.node_->get_parameter("debug").as_bool();

  // Same pattern as `debug`: the YAML `iterations` is the default; an `iterations` ROS param
  // (set by conveyor_feeding.launch.py) overrides it. The launch sentinel -1 means "keep the
  // YAML value", so only a launch override of >= 0 changes total_iterations.
  if (!application.node_->has_parameter("iterations")) {
    application.node_->declare_parameter("iterations", total_iterations);
  }
  int iterations_param = static_cast<int>(application.node_->get_parameter("iterations").as_int());
  if (iterations_param >= 0) {
    total_iterations = iterations_param;
  }
  RCLCPP_INFO(LOGGER, "conveyor_feeding will run %d iteration(s)", total_iterations);

  // Same pattern: YAML `max_pick_attempts` is the default; a ROS param overrides it. Bounds the
  // pick retries on a failed grasp (each failed attempt does a safe retreat first). 1 = no retry.
  if (!application.node_->has_parameter("max_pick_attempts")) {
    application.node_->declare_parameter("max_pick_attempts", max_pick_attempts);
  }
  max_pick_attempts = static_cast<int>(application.node_->get_parameter("max_pick_attempts").as_int());
  if (max_pick_attempts < 1) max_pick_attempts = 1;
  RCLCPP_INFO(LOGGER, "conveyor_feeding: max_pick_attempts = %d", max_pick_attempts);

  // Insertion strategy: YAML default, overridable by the launch-provided `insertion_mode`
  // param (same pattern as `debug`). "dp" = NN velocity segment; "moveit" = comparison test.
  if (!application.node_->has_parameter("insertion_mode")) {
    application.node_->declare_parameter("insertion_mode", insertion_mode);
  }
  insertion_mode = application.node_->get_parameter("insertion_mode").as_string();
  RCLCPP_INFO(LOGGER, "conveyor_feeding: insertion_mode = %s", insertion_mode.c_str());

  // Grasp-aware insertion: YAML default, overridable by the launch-provided
  // `grasp_aware_insertion` param (same pattern as `insertion_mode`).
  if (!application.node_->has_parameter("grasp_aware_insertion")) {
    application.node_->declare_parameter("grasp_aware_insertion", grasp_aware_insertion);
  }
  grasp_aware_insertion = application.node_->get_parameter("grasp_aware_insertion").as_bool();
  RCLCPP_INFO(LOGGER, "conveyor_feeding: grasp_aware_insertion = %s",
              grasp_aware_insertion ? "true" : "false");

  // Pick-depth flush + radius-aware insert: YAML defaults, overridable by launch params
  // (same pattern as grasp_aware_insertion).
  if (!application.node_->has_parameter("pick_depth_flush")) {
    application.node_->declare_parameter("pick_depth_flush", pick_depth_flush);
  }
  pick_depth_flush = application.node_->get_parameter("pick_depth_flush").as_bool();
  if (!application.node_->has_parameter("moveit_insert_radius_aware")) {
    application.node_->declare_parameter("moveit_insert_radius_aware", moveit_insert_radius_aware);
  }
  moveit_insert_radius_aware = application.node_->get_parameter("moveit_insert_radius_aware").as_bool();
  RCLCPP_INFO(LOGGER, "conveyor_feeding: pick_depth_flush = %s (compliance %.4f m), moveit_insert_radius_aware = %s "
              "(bottle_radius %.4f, grip_offset %.4f, suction_tip_compliance %.4f, seat_cup_stretch %.4f)",
              pick_depth_flush ? "true" : "false", pick_depth_compliance,
              moveit_insert_radius_aware ? "true" : "false",
              bottle_radius, grip_offset, suction_tip_compliance, seat_cup_stretch);

  // socket_pose_topic + MoveIt-mode geometry are YAML-only (no launch override needed).
  std::array<double, 3> insert_offset = {0.0, 0.0, 0.0};
  for (size_t i = 0; i < 3 && i < moveit_insert_offset.size(); ++i) insert_offset[i] = moveit_insert_offset[i];
  std::array<double, 4> insert_orientation = {0.515881, 0.483598, -0.515881, -0.483598};
  for (size_t i = 0; i < 4 && i < moveit_insert_orientation.size(); ++i) insert_orientation[i] = moveit_insert_orientation[i];
  if (insertion_mode == "moveit") {
    RCLCPP_INFO(LOGGER, "conveyor_feeding: moveit insert offset=[%.4f, %.4f, %.4f] above_dz=%.3f orient(xyzw)=[%.4f, %.4f, %.4f, %.4f] descent_collision_check=%s socket_topic=%s",
                insert_offset[0], insert_offset[1], insert_offset[2], moveit_insert_above_dz,
                insert_orientation[0], insert_orientation[1], insert_orientation[2], insert_orientation[3],
                moveit_insert_descent_collision_check ? "true" : "false", socket_pose_topic.c_str());
    RCLCPP_INFO(LOGGER, "conveyor_feeding: moveit insert fallback guards: max_waypoints=%d validate_descent=%s",
                moveit_insert_fallback_max_waypoints, moveit_insert_validate_descent ? "true" : "false");
  }

  std::array<double, 3> ga_bottle_offset = {0.0, 0.0, 0.078};
  for (size_t i = 0; i < 3 && i < grasp_aware_bottle_offset.size(); ++i) ga_bottle_offset[i] = grasp_aware_bottle_offset[i];

  // Named poses: apply the `pose_set` preset + any per-pose overrides on top of the YAML
  // values, and log the resolved set (same declare-if-absent pattern as `insertion_mode`).
  edi_bottle_picking::apply_pose_overrides(application.node_, scenario_poses, "conveyor_feeding");

  ConveyorFeedingUtils conveyor_feeding_utils(manipulator, grasp_pose_topic, default_controller, debug, is_isaac, max_pick_attempts,
                                              insertion_mode, socket_pose_topic, insert_offset, moveit_insert_above_dz, insert_orientation,
                                              moveit_insert_descent_collision_check,
                                              moveit_insert_fallback_max_waypoints, moveit_insert_validate_descent,
                                              grasp_aware_insertion, in_hand_pose_topic, ga_bottle_offset,
                                              pick_depth_flush, pick_depth_compliance, moveit_insert_radius_aware,
                                              bottle_radius, grip_offset, suction_tip_compliance, seat_cup_stretch,
                                              scenario_poses);

  rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
  if(!application.gripper_action_client_ptr->wait_for_action_server(d.to_chrono<std::chrono::duration<double>>())) {
    RCLCPP_INFO(LOGGER, "waiting for gripper server to come up.");
  }

  application.moveit_visual_tools_->deleteAllMarkers();

  application.move_group_ptr->setStartState(*application.move_group_ptr->getCurrentState());

  RCLCPP_INFO(LOGGER, "PLANNER FRAME: %s", application.move_group_ptr->getPlanningFrame().c_str());

  conveyor_feeding_utils.add_box();

  // Move once to the scenario's initial/home pose (wait_slam) before the cycle starts. This is
  // no longer re-visited each iteration -- iterations begin at above_box_1 directly. Best-effort:
  // warn (don't abort) on failure, since the first iteration's above_box_1 move runs regardless.
  if (!conveyor_feeding_utils.move_to_initial_pose()) {
    RCLCPP_WARN(LOGGER, "Could not reach initial pose 'wait_slam' at startup; continuing anyway");
  }

  bool success;
  int iter_count = 0;

  while((iter_count < total_iterations) && rclcpp::ok() && !sigint_received)
  {
    RCLCPP_INFO(LOGGER, "Starting iteration %d out of %d", iter_count+1, total_iterations);
    application.moveit_visual_tools_->deleteAllMarkers();
    success = conveyor_feeding_utils.run();
    if (success){
        RCLCPP_INFO(LOGGER, "Iteration %d successful", iter_count+1);
    } else {
        // The loop always advances iter_count, even on a failed cycle, so surface every
        // such failure with a clearly identifiable error line tied to the iteration number.
        RCLCPP_ERROR(LOGGER, "Iteration %d FAILED (see errors above); advancing to next iteration",
                     iter_count+1);
    }
    iter_count++;
  }

  rclcpp::shutdown();

  return 0;
}
