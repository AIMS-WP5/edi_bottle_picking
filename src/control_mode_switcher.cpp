#include <edi_bottle_picking/control_mode_switcher.h>

#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace edi_bottle_picking
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("control_mode_switcher");

ControlModeSwitcher::ControlModeSwitcher(rclcpp::Node::SharedPtr node,
                                         bool is_isaac,
                                         std::string position_controller,
                                         std::string velocity_controller)
    : node_(node), is_isaac_(is_isaac),
      position_controller_(std::move(position_controller)),
      velocity_controller_(std::move(velocity_controller)),
      dp_done_received_(false), dp_done_value_(false)
{
    velocity_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/velocity_control_mode/command");
    dp_start_pub_ = node_->create_publisher<std_msgs::msg::Empty>("dp_exec_start", 10);
    dp_done_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "dp_exec_done", 10, std::bind(&ControlModeSwitcher::dp_done_callback, this, _1));
    can_update_socket_pub_ = node_->create_publisher<std_msgs::msg::Bool>("can_update_socket", 10);

    RCLCPP_INFO(LOGGER, "ControlModeSwitcher: position='%s' velocity='%s' isaac=%s",
                position_controller_.c_str(), velocity_controller_.c_str(),
                is_isaac_ ? "true" : "false");
}

bool ControlModeSwitcher::to_velocity_control()
{
    // Controller switch first, then (Isaac) gain flip -- ordering verified on the bench.
    bool ok = switch_controllers(velocity_controller_, position_controller_);
    if (is_isaac_) {
        set_isaac_velocity_mode(true);
    }
    return ok;
}

bool ControlModeSwitcher::to_position_control()
{
    // Activate the trajectory controller first (it latches the current pose), then
    // restore position-control gains so the arm holds against that fresh target.
    bool ok = switch_controllers(position_controller_, velocity_controller_);
    if (is_isaac_) {
        set_isaac_velocity_mode(false);
    }
    return ok;
}

std::optional<bool> ControlModeSwitcher::run_dp_segment(double timeout_sec)
{
    {
        std::lock_guard<std::mutex> lock(dp_mutex_);
        dp_done_received_ = false;
    }

    // Freeze the external socket/target provider for the duration of the segment so the
    // insertion target is sampled once and cannot drift while DP is executing.
    publish_can_update_socket(false);

    if (!to_velocity_control()) {
        RCLCPP_ERROR(LOGGER, "DP segment: failed to switch to velocity control; aborting");
        to_position_control();  // best-effort restore
        publish_can_update_socket(true);
        return std::nullopt;
    }

    auto start_msg = std_msgs::msg::Empty();
    dp_start_pub_->publish(start_msg);
    RCLCPP_INFO(LOGGER, "DP segment: published /dp_exec_start, waiting up to %.1fs for /dp_exec_done",
                timeout_sec);

    std::optional<bool> result;
    {
        std::unique_lock<std::mutex> lock(dp_mutex_);
        // Measure the timeout on the NODE clock (which honors use_sim_time), not the wall
        // clock. std::condition_variable::wait_for is steady/wall-clock only, so under Isaac
        // (sim running slower than real time) it tripped spuriously: the DP segment is ~8 s of
        // SIM time but can take 20+ s of wall time, blowing a 20 s wall budget. With the node
        // clock, `timeout_sec` is sim seconds, so the budget tracks the DP node's own timeline.
        // wait_for here is just a bounded sleep between deadline re-checks; dp_done_callback's
        // notify still wakes us immediately when /dp_exec_done arrives. On the real robot
        // (use_sim_time:=false) the node clock IS wall time, so behaviour is unchanged there.
        const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_sec);
        while (rclcpp::ok() && !dp_done_received_ && node_->now() < deadline) {
            dp_cv_.wait_for(lock, std::chrono::milliseconds(100));
        }
        if (dp_done_received_) {
            result = dp_done_value_;
        }
    }

    if (!result.has_value()) {
        RCLCPP_WARN(LOGGER, "DP segment: timed out waiting for /dp_exec_done (%.1fs sim-time budget)",
                    timeout_sec);
    } else {
        RCLCPP_INFO(LOGGER, "DP segment: /dp_exec_done = %s", *result ? "success" : "failure");
    }

    // Always restore position control, even on timeout.
    to_position_control();
    // Release the socket/target provider now that the segment is done and position
    // control is restored.
    publish_can_update_socket(true);
    return result;
}

bool ControlModeSwitcher::switch_controllers(const std::string & activate,
                                             const std::string & deactivate)
{
    // Use a short-lived node so we can spin_until_future_complete without contending with
    // the executor that is already spinning node_ on another thread. Inherit the parent's
    // clock source (sim time off Isaac's /clock when use_sim_time) for consistency.
    rclcpp::NodeOptions sim_opts;
    sim_opts.parameter_overrides({
        rclcpp::Parameter("use_sim_time", node_->get_parameter("use_sim_time").as_bool())});
    auto tmp_node = std::make_shared<rclcpp::Node>("tmp_controller_switch_node", sim_opts);
    auto client = tmp_node->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");

    while (!client->wait_for_service(2s)) {
        RCLCPP_WARN(LOGGER, "Waiting for /controller_manager/switch_controller service...");
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = std::vector<std::string>{activate};
    request->deactivate_controllers = std::vector<std::string>{deactivate};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    builtin_interfaces::msg::Duration timeout;
    timeout.sec = 5;
    timeout.nanosec = 0;
    request->timeout = timeout;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(tmp_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to call /controller_manager/switch_controller");
        return false;
    }
    if (future.get()->ok) {
        RCLCPP_INFO(LOGGER, "Controller switch successful: +%s -%s",
                    activate.c_str(), deactivate.c_str());
        return true;
    }
    RCLCPP_ERROR(LOGGER, "Controller switch failed: +%s -%s", activate.c_str(), deactivate.c_str());
    return false;
}

void ControlModeSwitcher::set_isaac_velocity_mode(bool velocity)
{
    if (!velocity_mode_client_->service_is_ready()) {
        RCLCPP_WARN(LOGGER, "Isaac velocity-mode service '/velocity_control_mode/command' not "
                            "available; skipping gain flip to %s control",
                    velocity ? "VELOCITY" : "POSITION");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = velocity;
    // Fire-and-forget; the response is handled on the spinning executor thread.
    velocity_mode_client_->async_send_request(
        request,
        [velocity](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(LOGGER, "Isaac drive mode -> %s : success=%d (%s)",
                        velocity ? "VELOCITY" : "POSITION",
                        response->success, response->message.c_str());
        });
}

void ControlModeSwitcher::dp_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(dp_mutex_);
        dp_done_received_ = true;
        dp_done_value_ = msg->data;
    }
    dp_cv_.notify_all();
}

void ControlModeSwitcher::publish_can_update_socket(bool can_update)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = can_update;
    can_update_socket_pub_->publish(msg);
    RCLCPP_INFO(LOGGER, "Published /can_update_socket = %s (%s socket/target provider)",
                can_update ? "true" : "false", can_update ? "release" : "freeze");
}

}  // namespace edi_bottle_picking
