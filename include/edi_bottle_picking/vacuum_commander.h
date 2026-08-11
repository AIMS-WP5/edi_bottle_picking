#ifndef VACUUM_COMMANDER_H_
#define VACUUM_COMMANDER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <manipulator_interface/manipulator_interface.h>

#include <string>

namespace edi_bottle_picking
{

/** \brief Resolved backend flags for a scenario node.
 *
 *  \c simulation -- Isaac Sim is the PRIMARY plant (no UR driver): skip the UR /set_io
 *  vacuum path, trust the sim for grasp status, and flip the Isaac joint-drive gains on
 *  control-mode switches.
 *  \c mirror_to_isaac -- an Isaac instance is attached (primary OR follower/mirror):
 *  best-effort forward every vacuum command to Isaac's /vacuum_gripper/command bridge. */
struct BackendFlags
{
    bool simulation;
    bool mirror_to_isaac;
};

/** \brief Resolve the backend flags from tri-state ("auto"|"true"|"false") sources.
 *
 *  Precedence per flag: ROS parameter (launch arg) if not "auto", else the config-yaml
 *  value passed in (use "auto" when the key is absent), else the fallback:
 *  - \c simulation falls back to the node's use_sim_time (the historical heuristic), with
 *    a one-time WARN -- correct for Isaac-vs-URSim/real today, but set it explicitly for
 *    bag-replay / Gazebo / hybrid deployments.
 *  - \c mirror_to_isaac falls back to the resolved \c simulation value: Isaac-primary
 *    bring-ups mirror automatically (the bridge is the only vacuum there), ROS-only runs
 *    get no mirror with zero config. Hybrid bring-ups (URSim/real primary + Isaac
 *    follower) pass mirror_to_isaac:=true explicitly. */
BackendFlags resolve_backend_flags(const rclcpp::Node::SharedPtr & node,
                                   const std::string & config_simulation,
                                   const std::string & config_mirror_to_isaac,
                                   const rclcpp::Logger & logger);

/** \brief Facade for the vacuum gripper across real / URSim / Isaac backends.
 *
 *  One shared implementation of the command path that used to be triplicated across the
 *  scenario utils: on \c simulation skip the UR IO path entirely (Isaac/TopicBasedSystem
 *  has no /set_io service; the suction is modelled by the Isaac bridge), otherwise drive
 *  the real UR gripper via ManipulatorInterface::activate_vacuum_gripper(); then, when
 *  \c mirror_to_isaac, forward the command to Isaac's /vacuum_gripper/command bridge
 *  best-effort (WARNs once, not per command, if the bridge is absent). */
class VacuumCommander
{
public:
    VacuumCommander(rclcpp::Node::SharedPtr node,
                    manipulator_interface::ManipulatorInterface & manipulator,
                    const BackendFlags & flags);

    /** \brief Grip (true) or release (false). Returns the UR IO result on real backends,
     *  true on simulation (the sim bridge is authoritative there). */
    bool command(bool grip);

private:
    void mirror_to_isaac(bool grip);

    rclcpp::Node::SharedPtr node_;
    manipulator_interface::ManipulatorInterface & manipulator_;
    BackendFlags flags_;
    bool warned_bridge_absent_ = false;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr isaac_vacuum_client_;  // null unless mirroring
};

}  // namespace edi_bottle_picking

#endif /* VACUUM_COMMANDER_H_ */
