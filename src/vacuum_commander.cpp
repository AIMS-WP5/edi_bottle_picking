#include <edi_bottle_picking/vacuum_commander.h>

namespace edi_bottle_picking
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("vacuum_commander");

namespace
{
/** Tri-state parse: "true"/"false" are explicit, anything else means "auto" (unset). */
std::optional<bool> parse_tristate(const std::string & value)
{
    if (value == "true") return true;
    if (value == "false") return false;
    return std::nullopt;
}

/** ROS param (launch arg) if set and not "auto", else the config-yaml value. */
std::optional<bool> resolve_tristate(const rclcpp::Node::SharedPtr & node,
                                     const std::string & param_name,
                                     const std::string & config_value)
{
    std::string param_value = "auto";
    if (node->has_parameter(param_name)) {
        param_value = node->get_parameter(param_name).as_string();
    }
    auto resolved = parse_tristate(param_value);
    if (resolved.has_value()) {
        return resolved;
    }
    return parse_tristate(config_value);
}
}  // namespace

BackendFlags resolve_backend_flags(const rclcpp::Node::SharedPtr & node,
                                   const std::string & config_simulation,
                                   const std::string & config_mirror_to_isaac,
                                   const rclcpp::Logger & logger)
{
    BackendFlags flags;

    auto simulation = resolve_tristate(node, "simulation", config_simulation);
    if (simulation.has_value()) {
        flags.simulation = *simulation;
    } else {
        // Historical heuristic: Isaac is the only sim-time backend in this stack (URSim and
        // the real cell run wall clock, and the runbook forbids sim time against URSim).
        flags.simulation = node->get_parameter("use_sim_time").as_bool();
        RCLCPP_WARN(logger, "'simulation' not set; inferring simulation=%s from use_sim_time. "
                            "Set it explicitly for bag-replay/Gazebo/hybrid deployments.",
                    flags.simulation ? "true" : "false");
    }

    auto mirror = resolve_tristate(node, "mirror_to_isaac", config_mirror_to_isaac);
    // Default: follow the resolved simulation value. Isaac-primary mirrors automatically
    // (the bridge IS the vacuum there); ROS-only gets no mirror client and no WARN spam.
    // Hybrid (real/URSim primary + Isaac follower) passes mirror_to_isaac:=true explicitly.
    flags.mirror_to_isaac = mirror.has_value() ? *mirror : flags.simulation;

    RCLCPP_INFO(logger, "Backend flags: simulation=%s mirror_to_isaac=%s",
                flags.simulation ? "true" : "false", flags.mirror_to_isaac ? "true" : "false");
    return flags;
}

VacuumCommander::VacuumCommander(rclcpp::Node::SharedPtr node,
                                 manipulator_interface::ManipulatorInterface & manipulator,
                                 const BackendFlags & flags)
    : node_(node), manipulator_(manipulator), flags_(flags)
{
    if (flags_.mirror_to_isaac) {
        isaac_vacuum_client_ = node_->create_client<std_srvs::srv::SetBool>("/vacuum_gripper/command");
    }
}

bool VacuumCommander::command(bool grip)
{
    bool ok;
    if (flags_.simulation) {
        // Isaac/TopicBasedSystem has no UR /set_io service, so activate_vacuum_gripper()
        // would block on it and report the grasp failed (aborting the pick). In sim the
        // suction is modelled entirely by the Isaac bridge, so skip the UR IO path.
        ok = true;
    } else {
        // Drive the real UR vacuum gripper (via UR IO) -- real robot and URSim.
        ok = manipulator_.activate_vacuum_gripper(grip);
    }
    if (flags_.mirror_to_isaac) {
        mirror_to_isaac(grip);
    }
    return ok;
}

void VacuumCommander::mirror_to_isaac(bool grip)
{
    if (!isaac_vacuum_client_->service_is_ready()) {
        if (!warned_bridge_absent_) {
            RCLCPP_WARN(LOGGER, "Isaac vacuum service '/vacuum_gripper/command' not available; "
                                "skipping sim %s command (mirroring stays enabled; further "
                                "occurrences logged at DEBUG)", grip ? "GRIP" : "RELEASE");
            warned_bridge_absent_ = true;
        } else {
            RCLCPP_DEBUG(LOGGER, "Isaac vacuum bridge still absent; skipping sim %s command",
                         grip ? "GRIP" : "RELEASE");
        }
        return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = grip;
    isaac_vacuum_client_->async_send_request(
        request,
        [grip](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(LOGGER, "Isaac vacuum %s -> success=%d (%s)",
                        grip ? "GRIP" : "RELEASE", response->success, response->message.c_str());
        });
}

}  // namespace edi_bottle_picking
