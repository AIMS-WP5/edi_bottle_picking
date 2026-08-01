#include <edi_bottle_picking/scenario_poses.h>

namespace edi_bottle_picking
{

namespace
{

/** Read one optional string key, leaving the caller's default in place when absent. */
void read_key(const YAML::Node & config, const std::string & key, std::string & out)
{
    if (config[key]) {
        out = config[key].as<std::string>();
    }
}

/** Declare-if-absent, then read -- the pattern the scenarios already use for insertion_mode
    (the parameter may already exist when the launch file passed it). */
std::string param_or(const rclcpp::Node::SharedPtr & node, const std::string & name,
                     const std::string & fallback)
{
    if (!node->has_parameter(name)) {
        node->declare_parameter(name, fallback);
    }
    return node->get_parameter(name).as_string();
}

} // namespace

ScenarioPoses load_scenario_poses(const YAML::Node & config)
{
    ScenarioPoses poses;
    read_key(config, "pose_initial",          poses.initial);
    read_key(config, "pose_above_box",        poses.above_box);
    read_key(config, "pose_after_pickup",     poses.after_pickup);
    read_key(config, "pose_dp_handoff",       poses.dp_handoff);
    read_key(config, "pose_retreat_fallback", poses.retreat_fallback);
    read_key(config, "pose_dropoff",          poses.dropoff);
    return poses;
}

void apply_pose_overrides(const rclcpp::Node::SharedPtr & node, ScenarioPoses & poses,
                          const std::string & logger_name)
{
    const rclcpp::Logger logger = rclcpp::get_logger(logger_name);

    // pose_set first: a whole-set preset that individual overrides below can then adjust.
    // Empty means "leave the YAML values alone".
    const std::string pose_set = param_or(node, "pose_set", "");
    if (pose_set == "isaac") {
        // The legacy edi_isaacsim box (tool0 on the -Y side). dp_handoff is deliberately NOT
        // touched -- ai_start2 is canonical for the DP policy in both cells.
        poses.initial          = "wait_slam";
        poses.above_box        = "above_box_1";
        poses.after_pickup     = "ai_after_pickup";
        poses.retreat_fallback = "wait_slam";
    } else if (pose_set == "edi") {
        poses.initial          = "near_box";
        poses.above_box        = "above_box_2";
        poses.after_pickup     = "near_box";
        poses.retreat_fallback = "near_box";
    } else if (!pose_set.empty()) {
        RCLCPP_WARN(logger, "unrecognised pose_set '%s' (expected: edi | isaac); "
                            "using the configured poses unchanged", pose_set.c_str());
    }

    poses.initial          = param_or(node, "pose_initial",          poses.initial);
    poses.above_box        = param_or(node, "pose_above_box",        poses.above_box);
    poses.after_pickup     = param_or(node, "pose_after_pickup",     poses.after_pickup);
    poses.dp_handoff       = param_or(node, "pose_dp_handoff",       poses.dp_handoff);
    poses.retreat_fallback = param_or(node, "pose_retreat_fallback", poses.retreat_fallback);
    poses.dropoff          = param_or(node, "pose_dropoff",          poses.dropoff);

    // One line, always: picking the wrong cell's pose set does not fail cleanly (the arm just
    // sweeps to the other side of the robot and finds nothing), so make it visible up front.
    RCLCPP_INFO(logger, "poses: set=%s initial=%s above_box=%s after_pickup=%s "
                        "dp_handoff=%s retreat_fallback=%s dropoff=%s",
                pose_set.empty() ? "(yaml)" : pose_set.c_str(),
                poses.initial.c_str(), poses.above_box.c_str(), poses.after_pickup.c_str(),
                poses.dp_handoff.c_str(), poses.retreat_fallback.c_str(),
                poses.dropoff.c_str());
}

} // namespace edi_bottle_picking
