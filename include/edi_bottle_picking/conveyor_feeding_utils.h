#ifndef CONVEYOR_FEEDING_UTILS_
#define CONVEYOR_FEEDING_UTILS_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <chrono>
#include <memory>
#include <atomic>
#include <thread>
#include <optional>
#include <array>


namespace conveyor_feeding_utils
{
	class ConveyorFeedingUtils
    {
    public:
    ConveyorFeedingUtils(manipulator_interface::ManipulatorInterface& manipulator, std::string grasp_pose_topic, std::string default_controller, bool debug = false, bool is_isaac = false, int max_pick_attempts = 3,
                         std::string insertion_mode = "dp", std::string socket_pose_topic = "socket_center",
                         std::array<double, 3> moveit_insert_offset = {0.0, 0.0, 0.0}, double moveit_insert_above_dz = 0.10,
                         std::array<double, 4> moveit_insert_orientation = {0.515881, 0.483598, -0.515881, -0.483598},
                         bool moveit_insert_descent_collision_check = true); // Constructor

    ~ConveyorFeedingUtils(); // Destructor

    bool run();

    /** \brief Function to get first grasp position published by selected topic
        \param topic_name which topic to listen to
        \param stamped_topic if the topic is of type geometry_msgs::msg::PoseStamped, otherwise listens for geometry_msgs::msg::Pose
        \param timeout_sec how many seconds to wait before timeout
    */
    geometry_msgs::msg::Pose get_next_published_pose(std::string topic_name, bool stamped_topic, int timeout_sec = 5);

    /** \brief Function to check if the grasp pose is not off the vertical axis by more than the passed limit, and correct the pose if it is*/
    geometry_msgs::msg::Pose check_pose_angle(geometry_msgs::msg::Pose pose, double limit_deg = 30);

    geometry_msgs::msg::Pose get_curr_grasp_pose();
    void grasp_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void socket_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool add_box();

    bool get_grasped_status(int timeout_sec = 5);

	private:
        /** \brief Command the vacuum gripper: in sim (is_isaac) skips the UR /set_io path
            and mirrors the command to Isaac Sim's vacuum bridge; on real hardware drives the
            UR gripper. Returns the gripper result. */
        bool command_vacuum(bool grip);
        /** \brief Best-effort SetBool call to the Isaac vacuum bridge (no-op if absent). */
        void set_isaac_vacuum(bool grip);

        /** \brief Debug step-gate. When debug is enabled (constructor arg, live-toggled via
            /conveyor_feeding/debug), block until the user clicks 'Next' in the RViz
            RvizVisualToolsGui panel. The wait is interruptible: toggling debug off frees the
            run mid-wait with no final click. No-op when debug is disabled. */
        void maybe_prompt(const std::string& msg);

        /** \brief One full pick attempt: go to wait_slam, read the grasp pose, approach,
            descend, grip, confirm via get_grasped_status, and retreat to above_box_1. Returns
            true only if the bottle is grasped and the arm is back at above_box_1. */
        bool try_pick_bottle();

        /** \brief Best-effort recovery to a safe, plannable pose after a failed pick: release
            and detach any partial grasp, lift the tool straight up out of the box (a vertical
            Cartesian move -- a joint-space plan would arc out through a wall and fail), then
            return to above_box_1 (falling back to wait_slam). Prevents one failed pick from
            wedging the robot inside the box and bricking all following iterations. */
        bool safe_retreat();

        /** \brief MoveIt comparison insertion (alternative to ControlModeSwitcher::run_dp_segment).
            Reuses the can_update_socket freeze/release handshake but does NOT switch the
            controller/driver (stays in position control): reads socket_center once, plans a
            joint-space move to a pose above socket_center+offset (pose_goal), then a straight
            Cartesian descent (cartesian_goal). The bottle stays gripped; run() does the
            vacuum-off/detach afterwards exactly as for the DP path. Mirrors run_dp_segment's
            return: true/false = success/failure, std::nullopt = no socket target. */
        std::optional<bool> run_moveit_insert_segment(int socket_timeout_sec = 5);

        /** \brief Compute an IK solution (via the /compute_ik service) for an EE pose, seeded
            from the current arm config so the returned branch is the natural one nearest the
            current pose. Used to reach a pose via a joint-space move on the natural branch
            instead of a Cartesian move, which (jump_threshold=0) can route through a contorted,
            near-singular branch the controller then fails to track.
            \param max_seed_delta if > 0, reject a solution whose max per-joint deviation from the
                current config exceeds this (a flipped/contorted IK branch); <= 0 disables the
                check. Used for the fixed-orientation insert and the pick approach (see ignore_wrist).
            \param ignore_wrist scope of the branch check. false (insert): check all 6 joints and,
                on rejection, retry once with a wrist-flip re-seed (the fixed orientation makes any
                large delta a bad branch). true (pick approach): the top-down grasp wrist is free to
                rotate, so check only the arm joints 0-2 (shoulder/elbow contortion) and fail clean
                on rejection (no wrist-flip retry -- it can't fix an arm branch; safe_retreat + the
                next pick attempt re-solve from a new config).
            \return the 6 arm joint values, or nullopt if IK/service failed. */
        std::optional<std::vector<double>> compute_ik_seeded(const geometry_msgs::msg::Pose& target,
                                                             double max_seed_delta = -1.0,
                                                             bool ignore_wrist = false);

        manipulator_interface::ManipulatorInterface& manipulator_;
        std::atomic<bool> debug_;                 // live-toggled via /conveyor_feeding/debug
        std::atomic<bool> next_pressed_{false};   // set by RViz 'Next' on /rviz_visual_tools_gui
        bool success_, simulation_;
        int max_pick_attempts_;                   // bounded pick retries (config: max_pick_attempts)
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        // Latest insertion target (socket_center), latched continuously like the grasp pose.
        // A member subscription (created in the ctor on the main thread) avoids creating a
        // subscription mid-run from a worker thread, which crashes the MultiThreadedExecutor.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_socket_pose_;
        geometry_msgs::msg::Pose curr_socket_pose_;
        std::atomic<bool> socket_received_{false};
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr debug_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gui_sub_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr isaac_vacuum_client_;
        rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;  // /compute_ik (move_group)
        geometry_msgs::msg::Pose curr_grasp_pose_;
        // frame_id of the last received grasp pose; empty -> assume the camera frame.
        std::string curr_grasp_frame_;
        std::string default_controller_;
        // Insertion strategy: "dp" (NN velocity segment) or "moveit" (comparison: MoveIt
        // position-controlled above-socket move + Cartesian descent). Selected via config.
        std::string insertion_mode_;
        std::string socket_pose_topic_;           // topic carrying the insertion target (socket_center)
        std::array<double, 3> moveit_insert_offset_;  // EE target = socket_center + this (world XYZ, m)
        double moveit_insert_above_dz_;           // height above the insert pose for the MoveIt approach (m)
        std::array<double, 4> moveit_insert_orientation_;  // fixed EE orientation for the insert, [x,y,z,w]
        bool moveit_insert_descent_collision_check_;       // collision-check the Cartesian descent?
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
	};

}

#endif /* CONVEYOR_FEEDING_UTILS_ */