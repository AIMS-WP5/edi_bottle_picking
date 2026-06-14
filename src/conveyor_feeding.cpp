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
  }

  ConveyorFeedingUtils conveyor_feeding_utils(manipulator, grasp_pose_topic, default_controller, debug, is_isaac, max_pick_attempts,
                                              insertion_mode, socket_pose_topic, insert_offset, moveit_insert_above_dz, insert_orientation,
                                              moveit_insert_descent_collision_check);

  rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
  if(!application.gripper_action_client_ptr->wait_for_action_server(d.to_chrono<std::chrono::duration<double>>())) {
    RCLCPP_INFO(LOGGER, "waiting for gripper server to come up.");
  }

  application.moveit_visual_tools_->deleteAllMarkers();

  application.move_group_ptr->setStartState(*application.move_group_ptr->getCurrentState());

  RCLCPP_INFO(LOGGER, "PLANNER FRAME: %s", application.move_group_ptr->getPlanningFrame().c_str());

  conveyor_feeding_utils.add_box();

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
