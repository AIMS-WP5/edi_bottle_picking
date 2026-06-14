#ifndef CONTROL_MODE_SWITCHER_H_
#define CONTROL_MODE_SWITCHER_H_

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>

namespace edi_bottle_picking
{

/** \brief Facade for switching the arm between trajectory (position) control and
 *  velocity control, plus the diffusion-policy (DP) start/done handshake.
 *
 *  A control-mode change is, on every backend, a controller_manager controller switch
 *  (joint_trajectory_controller <-> forward_velocity_controller).  On Isaac Sim it must
 *  ALSO flip the joint-drive gains (kp=0 for velocity, USD position gains otherwise) via
 *  the velocity_mode_bridge SetBool service -- the real robot / URSim do not need this
 *  (URPositionHardwareInterface drives velocity natively).  The Isaac gain-flip is
 *  best-effort and only attempted when \c is_isaac is true, so this class works
 *  unchanged across real / URSim / Isaac.
 */
class ControlModeSwitcher
{
public:
    ControlModeSwitcher(rclcpp::Node::SharedPtr node,
                        bool is_isaac,
                        std::string position_controller = "joint_trajectory_controller",
                        std::string velocity_controller = "forward_velocity_controller");

    /** \brief Activate the velocity controller, deactivate the position controller, then
     *  (Isaac only) flip the drive gains to velocity control.  Returns true on a
     *  successful controller switch. */
    bool to_velocity_control();

    /** \brief Activate the position controller, deactivate the velocity controller, then
     *  (Isaac only) restore position-control drive gains.  The trajectory controller is
     *  activated first so it latches the current pose -- restoring stiffness against a
     *  stale target would otherwise snap the arm. */
    bool to_position_control();

    /** \brief Full MoveIt->velocity DP handoff: to_velocity_control() -> publish
     *  /dp_exec_start -> wait for /dp_exec_done (up to \p timeout_sec) ->
     *  to_position_control().  Returns the DP result (true = success, false =
     *  collision/limits) or std::nullopt on timeout.  Position control is always
     *  restored, even on timeout or a failed velocity switch.
     *
     *  \p timeout_sec is measured on the node clock, so it is SIM seconds when
     *  use_sim_time is set (and wall seconds on the real robot).  This keeps the
     *  budget aligned with the DP node's own (sim-time-paced) timeline instead of
     *  tripping spuriously when Isaac runs slower than real time.
     *
     *  The segment is also bracketed by the can_update_socket handshake: the external
     *  socket/target-coordinate provider is told to freeze (false) before the velocity
     *  switch and released (true) once position control is restored, so the insertion
     *  target cannot drift mid-segment ("get socket pos once"). */
    std::optional<bool> run_dp_segment(double timeout_sec = 20.0);

    /** \brief Freeze the external socket/target-coordinate provider (publish
     *  can_update_socket=false), so the insertion target is sampled once and cannot drift
     *  while a segment executes. Companion to release_socket(). Used by insertion paths that
     *  do NOT switch the controller (e.g. the MoveIt comparison segment), so they reuse the
     *  same handshake as run_dp_segment() without a velocity/gain switch. */
    void freeze_socket();

    /** \brief Release the external socket/target-coordinate provider (publish
     *  can_update_socket=true): segment done, the provider may advance to the next target. */
    void release_socket();

private:
    bool switch_controllers(const std::string & activate, const std::string & deactivate);
    void set_isaac_velocity_mode(bool velocity);   // best-effort SetBool, no-op if absent
    void dp_done_callback(const std_msgs::msg::Bool::SharedPtr msg);
    /** \brief Publish the socket-coordinate freeze handshake on /can_update_socket: false
        latches the external provider for the DP segment, true releases it. Best-effort --
        no subscriber is required to be present. */
    void publish_can_update_socket(bool can_update);

    rclcpp::Node::SharedPtr node_;
    bool is_isaac_;
    std::string position_controller_;
    std::string velocity_controller_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr velocity_mode_client_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dp_start_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dp_done_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_update_socket_pub_;

    std::mutex dp_mutex_;
    std::condition_variable dp_cv_;
    bool dp_done_received_;
    bool dp_done_value_;
};

}  // namespace edi_bottle_picking

#endif /* CONTROL_MODE_SWITCHER_H_ */
