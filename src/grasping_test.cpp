#include <edi_bottle_picking/grasping_test.h>
#include <csignal>

using namespace grasping_test;
using namespace manipulator_interface;
using namespace grasping_test_utils;
using namespace std::chrono_literals;

std::atomic_bool sigint_received(false);

std::string file_path = __FILE__;
auto pos = file_path.find_last_of("/");
std::string file_dir_path = file_path.substr(0, pos);
std::string config_file_path = file_dir_path + "/../config/grasping_test_config.yaml";
YAML::Node config = YAML::LoadFile(config_file_path);
bool debug = config["debug"].as<bool>();
int total_iterations = config["iterations"].as<int>();
std::string grasp_pose_topic = config["grasp_pose_topic"].as<std::string>();

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  GraspingTest application;

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

  // Running against Isaac Sim is signalled by use_sim_time (set true by the launch
  // when connected to the sim clock). In that mode grasping_test skips hardware-only
  // steps (UR IO grasp check, real-controller switches).
  bool simulation = application.node_->get_parameter("use_sim_time").as_bool();
  // Whether to run the MoveIt->real-time (DP/PyTorch) controller switchover during pick.
  // Disable for model-less runs (e.g. plain pick/place in sim with no DP node).
  bool run_dp_switchover = application.node_->get_parameter_or("run_dp_switchover", true);
  RCLCPP_INFO(LOGGER, "grasping_test: simulation=%s, run_dp_switchover=%s",
              simulation ? "ON" : "OFF", run_dp_switchover ? "ON" : "OFF");

  GraspingTestUtils grasping_test_utils(manipulator, grasp_pose_topic, debug, simulation, run_dp_switchover);

  rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
  if(!application.gripper_action_client_ptr->wait_for_action_server(d.to_chrono<std::chrono::duration<double>>())) {
    RCLCPP_INFO(LOGGER, "waiting for gripper server to come up.");
  }

  application.moveit_visual_tools_->deleteAllMarkers();

  application.move_group_ptr->setStartState(*application.move_group_ptr->getCurrentState());

  RCLCPP_INFO(LOGGER, "PLANNER FRAME: %s", application.move_group_ptr->getPlanningFrame().c_str());

  grasping_test_utils.add_box();

  bool success;
  int iter_count = 0;

  while((iter_count < total_iterations) && rclcpp::ok() && !sigint_received)
  {
    RCLCPP_INFO(LOGGER, "Starting iteration %d out of %d", iter_count+1, total_iterations);
    application.moveit_visual_tools_->deleteAllMarkers();
    success = grasping_test_utils.pick_up();
    if (success){
        grasping_test_utils.put_down();
    }
    iter_count++;
  }

  rclcpp::shutdown();

  return 0;
}
