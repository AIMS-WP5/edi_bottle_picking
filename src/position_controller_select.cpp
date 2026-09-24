#include <edi_bottle_picking/position_controller_select.h>

namespace edi_bottle_picking
{

PositionControllerChoice select_position_controller(
    const std::vector<std::pair<std::string, std::string>> & loaded)
{
    static const char * const kCandidates[] = {
        "scaled_joint_trajectory_controller", "joint_trajectory_controller"};

    std::vector<std::pair<std::string, std::string>> found;   // candidates only, (name, state)
    for (const char * candidate : kCandidates) {
        for (const auto & [name, state] : loaded) {
            if (name == candidate) {
                found.emplace_back(name, state);
            }
        }
    }

    PositionControllerChoice choice;
    std::vector<std::pair<std::string, std::string>> active;
    for (const auto & entry : found) {
        if (entry.second == "active") {
            active.push_back(entry);
        }
    }
    if (active.size() == 1) {
        choice.name = active[0].first;
        choice.state = active[0].second;
        return choice;
    }
    if (active.empty() && found.size() == 1) {
        choice.name = found[0].first;
        choice.state = found[0].second;
        return choice;
    }

    if (found.empty()) {
        choice.error = "neither scaled_joint_trajectory_controller nor joint_trajectory_controller "
                       "is loaded";
    } else {
        choice.error = "ambiguous:";
        for (const auto & [name, state] : found) {
            choice.error += " " + name + " (" + state + ")";
        }
    }
    choice.error += "; set the position controller explicitly";
    return choice;
}

}  // namespace edi_bottle_picking
