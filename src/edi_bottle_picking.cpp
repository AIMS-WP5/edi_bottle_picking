#include <edi_bottle_picking/edi_bottle_picking.h>
#include <csignal>

using namespace edi_bottle_picking;
using namespace manipulator_interface;
using namespace edi_bottle_picking_utils;
using namespace std::chrono_literals;

std::atomic_bool sigint_received(false);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  EdiBottlePicking application;

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

  EdiBottlePickingUtils bottle_pick_utils(manipulator, true);

  rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
  if(!application.gripper_action_client_ptr->wait_for_action_server(d.to_chrono<std::chrono::duration<double>>())) {
    RCLCPP_INFO(LOGGER, "waiting for gripper server to come up.");
  }

  application.moveit_visual_tools_->deleteAllMarkers();

  application.move_group_ptr->setStartState(*application.move_group_ptr->getCurrentState());

  RCLCPP_INFO(LOGGER, "PLANNER FRAME: %s", application.move_group_ptr->getPlanningFrame().c_str());

  bool success;

  while(rclcpp::ok() && !sigint_received)
  {
    application.moveit_visual_tools_->deleteAllMarkers();
    success = bottle_pick_utils.pick_bottle();
    if (success){
        bottle_pick_utils.put_back_on_table();
    }
  }

  rclcpp::shutdown();

  return 0;
}
