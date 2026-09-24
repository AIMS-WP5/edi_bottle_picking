#ifndef POSITION_CONTROLLER_SELECT_H_
#define POSITION_CONTROLLER_SELECT_H_

#include <string>
#include <utility>
#include <vector>

namespace edi_bottle_picking
{

/** Value of default_controller / position_controller that asks ControlModeSwitcher to
 *  resolve the arm's trajectory controller from controller_manager at runtime. */
inline constexpr const char * kAutoPositionController = "auto";

/** \brief Result of select_position_controller(): \c name is set on success, \c error on
 *  failure (exactly one of the two is non-empty). \c state is the chosen controller's
 *  controller_manager state, for logging. */
struct PositionControllerChoice
{
    std::string name;
    std::string state;
    std::string error;
};

/** \brief Pick the arm's position (trajectory) controller from a controller_manager listing.
 *
 *  edi_ur loads exactly one of scaled_joint_trajectory_controller (real robot / URSim) and
 *  joint_trajectory_controller (Isaac / mock hardware); naming the other in a STRICT
 *  switch fails. Only those two candidates are considered:
 *    - exactly one of them active -> that one;
 *    - else exactly one of them loaded (any state; e.g. controller_stopper keeps the scaled
 *      JTC inactive until the robot program plays) -> that one;
 *    - else (both loaded, or neither) -> error listing what was found.
 *
 *  \p loaded is (name, state) for every controller controller_manager reports. */
PositionControllerChoice select_position_controller(
    const std::vector<std::pair<std::string, std::string>> & loaded);

}  // namespace edi_bottle_picking

#endif /* POSITION_CONTROLLER_SELECT_H_ */
