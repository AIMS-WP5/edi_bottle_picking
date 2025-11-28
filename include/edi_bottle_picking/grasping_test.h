#ifndef GRASPING_TEST_H_
#define GRASPING_TEST_H_

#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "control_msgs/action/gripper_command.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/grasping_test_utils.h>

#include "yaml-cpp/yaml.h"
#include <string>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
typedef boost::shared_ptr<tf2_ros::Buffer> tfBufferPtr;

typedef rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand> GoalHandleGripper;

namespace grasping_test
{
	class GraspingTest
	{
	public:
		GraspingTest()
		{
			node_options.use_intra_process_comms(false);
			node_options.automatically_declare_parameters_from_overrides(true);
			node_ = std::make_shared<rclcpp::Node>("grasping_test_node", node_options);

			moveit_visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link", "/moveit_visual_markers"));
			moveit_visual_tools_->loadPlanningSceneMonitor();
			moveit_visual_tools_->loadMarkerPub(true);
			moveit_visual_tools_->waitForMarkerSub();
			moveit_visual_tools_->loadRobotStatePub("display_robot_state");
			moveit_visual_tools_->setManualSceneUpdating();
			moveit_visual_tools_->loadRemoteControl();
			moveit_visual_tools_->deleteAllMarkers();

		}

		// ~GraspingTest(); // Destructor
		
		rclcpp::NodeOptions node_options;
		rclcpp::Node::SharedPtr node_;
		moveit_visual_tools::MoveItVisualToolsPtr moveit_visual_tools_;

		MoveGroupPtr move_group_ptr;
		rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_ptr;
		
		tfBufferPtr tf_buffer_ptr;
		tfListenerPtr tf_listener_ptr;		
	};
}


#endif /* GRASPING_TEST_H_ */