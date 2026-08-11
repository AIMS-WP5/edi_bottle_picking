// Pins the backend-flag resolution order (launch param -> config yaml -> fallback):
// `simulation` falls back to the historical use_sim_time heuristic, `mirror_to_isaac`
// follows the resolved simulation value unless set explicitly. A silent mix-up here
// either drives the real vacuum during a sim run or spams a ROS-only run with Isaac
// mirror traffic, so the precedence is worth a regression pin.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <edi_bottle_picking/vacuum_commander.h>

namespace
{

rclcpp::Node::SharedPtr make_node(bool use_sim_time,
                                  const std::string & simulation_param = "",
                                  const std::string & mirror_param = "")
{
    static int counter = 0;
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides = {rclcpp::Parameter("use_sim_time", use_sim_time)};
    if (!simulation_param.empty()) {
        overrides.emplace_back("simulation", simulation_param);
    }
    if (!mirror_param.empty()) {
        overrides.emplace_back("mirror_to_isaac", mirror_param);
    }
    options.parameter_overrides(overrides);
    options.automatically_declare_parameters_from_overrides(true);
    return std::make_shared<rclcpp::Node>("test_backend_flags_node_" + std::to_string(counter++),
                                          options);
}

edi_bottle_picking::BackendFlags resolve(const rclcpp::Node::SharedPtr & node,
                                         const std::string & yaml_simulation = "auto",
                                         const std::string & yaml_mirror = "auto")
{
    return edi_bottle_picking::resolve_backend_flags(node, yaml_simulation, yaml_mirror,
                                                     node->get_logger());
}

TEST(BackendFlags, FallbackFollowsUseSimTime)
{
    auto flags = resolve(make_node(false));
    EXPECT_FALSE(flags.simulation);
    EXPECT_FALSE(flags.mirror_to_isaac);

    flags = resolve(make_node(true));
    EXPECT_TRUE(flags.simulation);
    EXPECT_TRUE(flags.mirror_to_isaac);
}

TEST(BackendFlags, ExplicitParamBeatsHeuristic)
{
    // Sim-time run (e.g. bag replay) explicitly declared NOT to be Isaac-primary.
    auto flags = resolve(make_node(true, "false"));
    EXPECT_FALSE(flags.simulation);
    EXPECT_FALSE(flags.mirror_to_isaac);

    flags = resolve(make_node(false, "true"));
    EXPECT_TRUE(flags.simulation);
    EXPECT_TRUE(flags.mirror_to_isaac);
}

TEST(BackendFlags, YamlBeatsHeuristicParamBeatsYaml)
{
    auto flags = resolve(make_node(true), "false");
    EXPECT_FALSE(flags.simulation);

    // Param "auto" defers to yaml; param "false" overrides yaml "true".
    flags = resolve(make_node(false, "auto"), "true");
    EXPECT_TRUE(flags.simulation);
    flags = resolve(make_node(false, "false"), "true");
    EXPECT_FALSE(flags.simulation);
}

TEST(BackendFlags, MirrorIndependentWhenExplicit)
{
    // Hybrid shape: real/URSim primary with an Isaac follower attached.
    auto flags = resolve(make_node(false, "false", "true"));
    EXPECT_FALSE(flags.simulation);
    EXPECT_TRUE(flags.mirror_to_isaac);

    // Yaml can also pin the mirror independently of simulation.
    flags = resolve(make_node(false, "false"), "auto", "true");
    EXPECT_TRUE(flags.mirror_to_isaac);

    // And mirroring can be forced OFF even on an Isaac-primary run.
    flags = resolve(make_node(true, "true", "false"));
    EXPECT_TRUE(flags.simulation);
    EXPECT_FALSE(flags.mirror_to_isaac);
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
