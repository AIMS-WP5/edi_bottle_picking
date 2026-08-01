#ifndef EDI_BOTTLE_PICKING_SCENARIO_POSES_
#define EDI_BOTTLE_PICKING_SCENARIO_POSES_

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>

namespace edi_bottle_picking
{

/** \brief The named SRDF poses (``<group_state>`` in
    edi_ur/edi_moveit_config/srdf/ur5e_macro.srdf.xacro) a pick/insert scenario moves through.

    WHY THIS IS CONFIGURABLE, and not just literals in the scenario code: there are two
    incompatible sets of box-approach poses, belonging to two different physical cells, and
    they put the tool on OPPOSITE SIDES of the robot:

      "edi"   (default) -- above_box_2 / near_box. The real EDI cell, and equally the
                           robo-codegen-edi sim's bin (tool0 y ~ +0.5). This is where active
                           development is going.
      "isaac" (legacy)  -- above_box_1 / ai_after_pickup / wait_slam. The edi_isaacsim
                           scene's box (tool0 y ~ -0.47). edi_isaacsim is being DEPRECATED in
                           favour of robo-codegen-edi; this set exists so bringup_sim_stack.sh
                           keeps working through the deprecation window.

    Ground truth for the two sets' world poses is robo-codegen-edi's
    ur5e_common.py NAMED_JOINT_CONFIG_EE_POSES (baked FK over the same URDF cuMotion loads).

    Selecting the WRONG set does not fail cleanly -- the arm plans a long sweep to the other
    side of the cell and finds no bottle -- so the resolved set is logged once at startup. */
struct ScenarioPoses
{
    /** One-time startup / home pose, before the iteration loop. */
    std::string initial          = "near_box";
    /** Staging pose above the pick box: approach, and the retreat back out of it. */
    std::string above_box        = "above_box_2";
    /** Transit pose after a successful pick / after the insert cleanup moves. */
    std::string after_pickup     = "near_box";
    /** Canonical DP hand-off configuration. NOT part of the edi/isaac split and NOT
        overridden by pose_set: the DP policy was trained from this exact configuration, and
        in moveit insertion mode it seeds the insertion IK. Changing it silently degrades
        placement accuracy (iteration-8 matrix: a re-solved arrival was ~13 cm off). */
    std::string dp_handoff       = "ai_start2";
    /** safe_retreat()'s last resort when above_box is unreachable. */
    std::string retreat_fallback = "near_box";
    /** Where a bottle is dumped when the insertion failed. */
    std::string dropoff          = "inter_floor_4";
};

/** \brief Read the pose_* keys from a scenario's YAML config, falling back to the
    ScenarioPoses defaults (the "edi" set) for any key that is absent. */
ScenarioPoses load_scenario_poses(const YAML::Node & config);

/** \brief Apply ROS-parameter overrides on top of the YAML values, then log the resolved set.

    Follows the same declare-if-absent pattern the scenarios already use for insertion_mode:
      - `pose_set` ("edi" | "isaac") swaps the whole box-approach set in one go; this is what
        launch files and bringup scripts pass. An unrecognised value warns and is ignored.
      - `pose_initial`, `pose_above_box`, `pose_after_pickup`, `pose_dp_handoff`,
        `pose_retreat_fallback`, `pose_dropoff` override individual poses, and are applied
        AFTER pose_set so a preset can be adjusted one pose at a time.

    \param node the scenario node (parameters are declared on it if not already present)
    \param poses updated in place
    \param logger_name prefix for the single resolved-poses log line */
void apply_pose_overrides(const rclcpp::Node::SharedPtr & node, ScenarioPoses & poses,
                          const std::string & logger_name);

} // namespace edi_bottle_picking

#endif // EDI_BOTTLE_PICKING_SCENARIO_POSES_
