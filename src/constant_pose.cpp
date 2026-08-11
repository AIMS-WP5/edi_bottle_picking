#include <edi_bottle_picking/constant_pose.h>
#include <csignal>

using namespace constant_pose;
using namespace manipulator_interface;
using namespace constant_pose_utils;
using namespace std::chrono_literals;

std::atomic_bool sigint_received(false);

std::string file_path = __FILE__;
auto pos = file_path.find_last_of("/");
std::string file_dir_path = file_path.substr(0, pos);
std::string config_file_path = file_dir_path + "/../config/constant_pose_config.yaml";
YAML::Node config = YAML::LoadFile(config_file_path);
bool debug = config["debug"].as<bool>();
int total_iterations = config["iterations"].as<int>();
bool pose_from_topic = config["pose_from_topic"].as<bool>();
std::string pose_topic_name = config["pose_topic_name"].as<std::string>();
// Position controller to restore after the DP velocity segment (ca1e3b0 parity):
// scaled_joint_trajectory_controller on the real robot.
std::string default_controller = config["default_controller"]
    ? config["default_controller"].as<std::string>() : "joint_trajectory_controller";
// Named SRDF poses: YAML defaults here, `pose_set` / per-pose ROS params applied in main()
// once the node exists. See scenario_poses.h for the edi-vs-isaac cell split.
edi_bottle_picking::ScenarioPoses scenario_poses = edi_bottle_picking::load_scenario_poses(config);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ConstantPose application;

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

  // Backend flags: explicit `simulation` / `mirror_to_isaac` (launch arg > yaml > fallback:
  // simulation from use_sim_time with a WARN, mirror follows simulation). See vacuum_commander.h.
  edi_bottle_picking::BackendFlags backend = edi_bottle_picking::resolve_backend_flags(
      application.node_,
      config["simulation"] ? config["simulation"].as<std::string>() : "auto",
      config["mirror_to_isaac"] ? config["mirror_to_isaac"].as<std::string>() : "auto",
      LOGGER);
  edi_bottle_picking::apply_pose_overrides(application.node_, scenario_poses, "constant_pose");

  ConstantPoseUtils constant_pose_utils(manipulator, pose_from_topic, pose_topic_name, default_controller,
                                        debug, backend, scenario_poses);

  rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
  if(!application.gripper_action_client_ptr->wait_for_action_server(d.to_chrono<std::chrono::duration<double>>())) {
    RCLCPP_INFO(LOGGER, "waiting for gripper server to come up.");
  }

  application.moveit_visual_tools_->deleteAllMarkers();

  application.move_group_ptr->setStartState(*application.move_group_ptr->getCurrentState());

  RCLCPP_INFO(LOGGER, "PLANNER FRAME: %s", application.move_group_ptr->getPlanningFrame().c_str());

  bool success;
  int iter_count = 0;

  while((iter_count < total_iterations) && rclcpp::ok() && !sigint_received)
  {
    RCLCPP_INFO(LOGGER, "Starting iteration %d out of %d", iter_count+1, total_iterations);
    application.moveit_visual_tools_->deleteAllMarkers();
    success = constant_pose_utils.pickup();
    if (success) RCLCPP_INFO(LOGGER, "Iteration %d successful", iter_count+1);
    iter_count++;
  }

  rclcpp::shutdown();

  return 0;
}
