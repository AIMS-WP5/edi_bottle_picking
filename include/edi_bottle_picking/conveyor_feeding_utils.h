#ifndef CONVEYOR_FEEDING_UTILS_
#define CONVEYOR_FEEDING_UTILS_

#include <rclcpp/rclcpp.hpp>
#include <manipulator_interface/manipulator_interface.h>
#include <edi_bottle_picking/control_mode_switcher.h>
#include <edi_bottle_picking/scenario_poses.h>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <tf2/LinearMath/Transform.h>
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
                         bool moveit_insert_descent_collision_check = true,
                         int moveit_insert_fallback_max_waypoints = 85, bool moveit_insert_validate_descent = true,
                         bool grasp_aware_insertion = false, std::string in_hand_pose_topic = "grasp_in_hand",
                         std::array<double, 3> grasp_aware_bottle_offset = {0.0, 0.0, 0.078},
                         bool pick_depth_flush = false, double pick_depth_compliance = 0.0,
                         bool moveit_insert_radius_aware = false,
                         double bottle_radius = 0.0176, double grip_offset = 0.012,
                         double suction_tip_compliance = 0.0, double seat_cup_stretch = 0.0011,
                         edi_bottle_picking::ScenarioPoses poses = {}); // Constructor

    ~ConveyorFeedingUtils(); // Destructor

    /** \brief Move once to the scenario's initial/home pose (poses_.initial) before the
        iteration loop starts. That pose used to be re-visited at the start of every pick
        attempt; it is now only the one-time startup pose, so each iteration begins its motion
        at poses_.above_box directly. Best-effort: returns the plan result so the caller can
        warn (not abort) on failure -- the first iteration's above_box move is attempted
        regardless. */
    bool move_to_initial_pose();

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
    void in_hand_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool add_box();

    bool get_grasped_status(int timeout_sec = 5);

	private:
        /** \brief Command the vacuum gripper: in sim (is_isaac) skips the UR /set_io path
            and mirrors the command to Isaac Sim's vacuum bridge; on real hardware drives the
            UR gripper. Returns the gripper result. */
        bool command_vacuum(bool grip);
        /** \brief Best-effort SetBool call to the Isaac vacuum bridge (no-op if absent). */
        void set_isaac_vacuum(bool grip);

        /** \brief Debug step-gate; forwards to the shared manipulator_interface::DebugStepGate
            installed on manipulator_ by our constructor (topic /conveyor_feeding/debug).
            Kept as a thin forwarder so the existing call sites read unchanged. */
        void maybe_prompt(const std::string& msg);

        /** \brief One full pick attempt: read the grasp pose, move above the box
            (poses_.above_box), approach, descend, grip, confirm via get_grasped_status, and
            retreat to poses_.above_box. Returns true only if the bottle is grasped and the
            arm is back above the box. */
        bool try_pick_bottle();

        /** \brief Best-effort recovery to a safe, plannable pose after a failed pick: release
            and detach any partial grasp, lift the tool straight up out of the box (a vertical
            Cartesian move -- a joint-space plan would arc out through a wall and fail), then
            return to poses_.above_box (falling back to poses_.retreat_fallback). Prevents one failed pick from
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

        /** \brief Grasp-aware insertion poses: derive EE (virtual_ee_link) target CANDIDATES from
            the MEASURED bottle-in-hand transform (grasp_in_hand, published by Isaac at the suction
            bond) instead of the fixed calibrated orientation/offset. The desired BOTTLE pose is
            upright (local +Z up) at socket + grasp_aware_bottle_offset_; the spin about world Z is
            a FREE DOF (axis-symmetric bottle), so candidates at phi*, phi*+/-90, phi*+180 deg are
            returned ordered by EE-orientation closeness to the calibrated one (phi* = closest).
            The caller tries them in order against the seeded-IK + descent guards -- a mirrored
            grasp (flipped bottle) is typically IK-hostile at phi* but clean at phi*+180. The EE
            target is T_world_bottle * inverse(T_ee_bottle). With the canonical seat transform the
            first candidate reproduces the fixed-pose target (regression anchor, logged).
            \return candidate EE insert poses (best first), or empty if no valid in-hand pose. */
        std::vector<geometry_msgs::msg::Pose> compute_grasp_aware_insert_candidates(
            const geometry_msgs::msg::Pose& socket);

        /** \brief Core of the insert-candidate derivation, shared by the MEASURED (grasp-aware)
            and CANONICAL (radius-aware fixed-mode) paths: given the bottle-in-EE transform
            T_ee_bottle, return EE (virtual_ee_link) target candidates that place the bottle upright
            at socket + grasp_aware_bottle_offset_, with the free world-Z spin candidates ordered by
            closeness to the calibrated orientation. EE target = T_world_bottle * inverse(T_ee_bottle). */
        std::vector<geometry_msgs::msg::Pose> insert_candidates_from_bottle_in_ee(
            const geometry_msgs::msg::Pose& socket, const tf2::Transform& T_ee_bottle);

        /** \brief Build the CANONICAL bottle-in-EE (virtual_ee_link) transform analytically from
            the physical geometry constants (bottle_radius_, grip_offset_, suction_tip_compliance_,
            seat_cup_stretch_), for the radius-aware fixed-mode insertion. Fed through the SAME
            T_we = T_wb * inverse(T_ee_bottle) derivation as the measured grasp-aware path, so both
            modes share one radius-aware code path. Rotation = inverse(calibrated insert orientation)
            so the derived orientation reproduces the calibrated one at spin 0; origin places the EE
            origin at (radial_overhang, 0, grip_offset) in the bottle frame, radial_overhang =
            bottle_radius - suction_tip_compliance + seat_cup_stretch (= r + stretch since
            tool-tip-305: compliance is 0, the rigid tip IS virtual_ee_link). Roll about the bottle long
            axis is arbitrary (irrelevant) -- the free world-Z spin search selects the orientation.
            At the baseline bottle this reproduces moveit_insert_offset_xyz exactly (regression anchor). */
        tf2::Transform compute_canonical_in_hand_transform();

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
                                                             bool ignore_wrist = false,
                                                             const std::optional<std::vector<double>>& explicit_seed = std::nullopt);

        manipulator_interface::ManipulatorInterface& manipulator_;
        bool success_, simulation_;
        int max_pick_attempts_;                   // bounded pick retries (config: max_pick_attempts)
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_grasp_pose_;
        // Latest insertion target (socket_center), latched continuously like the grasp pose.
        // A member subscription (created in the ctor on the main thread) avoids creating a
        // subscription mid-run from a worker thread, which crashes the MultiThreadedExecutor.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_socket_pose_;
        geometry_msgs::msg::Pose curr_socket_pose_;
        std::atomic<bool> socket_received_{false};
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr isaac_vacuum_client_;
        rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;  // /compute_ik (move_group)
        geometry_msgs::msg::Pose curr_grasp_pose_;
        // frame_id of the last received grasp pose; empty -> assume the camera frame.
        std::string curr_grasp_frame_;
        std::string default_controller_;
        /** Named SRDF poses this scenario moves through; see scenario_poses.h for the
            edi (real cell / robo-codegen-edi) vs isaac (legacy edi_isaacsim) split. */
        edi_bottle_picking::ScenarioPoses poses_;
        // Insertion strategy: "dp" (NN velocity segment) or "moveit" (comparison: MoveIt
        // position-controlled above-socket move + Cartesian descent). Selected via config.
        std::string insertion_mode_;
        std::string socket_pose_topic_;           // topic carrying the insertion target (socket_center)
        std::array<double, 3> moveit_insert_offset_;  // EE target = socket_center + this (world XYZ, m)
        double moveit_insert_above_dz_;           // height above the insert pose for the MoveIt approach (m)
        std::array<double, 4> moveit_insert_orientation_;  // fixed EE orientation for the insert, [x,y,z,w]
        bool moveit_insert_descent_collision_check_;       // collision-check the Cartesian descent?
        // Guards on the above-socket move's pose_goal global-planner fallback (MoveIt mode only):
        int moveit_insert_fallback_max_waypoints_;         // reject (fail iter) if the fallback plan exceeds this many waypoints
        bool moveit_insert_validate_descent_;              // pre-validate the descent from the planned above-config before executing
        // Grasp-aware insertion (default off): derive the insert EE pose from the measured
        // bottle-in-hand transform instead of the fixed calibrated orientation/offset.
        bool grasp_aware_insertion_;
        std::string in_hand_pose_topic_;                   // topic carrying the measured in-hand transform
        // Desired BOTTLE-origin target relative to socket_center (world XYZ, m). Default
        // [0,0,0.078] is what the calibrated fixed-pose numbers imply: insert_offset_z (0.09)
        // minus the canonical grip_offset (0.012); XY centred (the 0.015 offset_x is exactly
        // the wrist->bottle overhang along the horizontal tool axis, cancelled in bottle terms).
        std::array<double, 3> grasp_aware_bottle_offset_;
        // --- Physical bottle / suction-tip geometry (source-of-truth; radius-aware pick+insert) ---
        bool pick_depth_flush_;            // press the pick to surface-flush (needs best_grasp at surface)
        double pick_depth_compliance_;     // press depth (m); default = suction_tip_compliance
        bool moveit_insert_radius_aware_;  // route fixed-mode insert through the canonical radius-aware transform
        double bottle_radius_;             // cup-contact radius of the gripped bottle (m)
        double grip_offset_;               // cup contact offset ALONG the bottle long axis (axial, m)
        double suction_tip_compliance_;    // 0.0 since tool-tip-305: the rigid cup tip IS at virtual_ee_link (0.305)
        double seat_cup_stretch_;          // cup stretch at bond (mirrors Isaac SEAT_CUP_STRETCH)
        // Latest in-hand pose (bottle in wrist_3_link frame). frame_id gates validity: Isaac
        // publishes "wrist_3_link" while a bottle is bonded and "none" otherwise.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_in_hand_pose_;
        geometry_msgs::msg::Pose curr_in_hand_pose_;
        std::string curr_in_hand_frame_;
        std::atomic<bool> in_hand_received_{false};
        std::unique_ptr<edi_bottle_picking::ControlModeSwitcher> control_switcher_;
	};

}

#endif /* CONVEYOR_FEEDING_UTILS_ */