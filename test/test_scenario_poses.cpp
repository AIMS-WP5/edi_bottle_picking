// Regression tests for the named-pose resolution (scenario_poses.h).
//
// Why these matter: the "edi" and "isaac" pose sets belong to two different physical cells and
// put the tool on OPPOSITE SIDES of the robot. Selecting the wrong one does not fail cleanly --
// the arm plans a long sweep to the other side and picks nothing -- so the resolution order
// (struct defaults -> YAML -> pose_set preset -> per-pose params) is worth pinning down.

#include <gtest/gtest.h>
#include <edi_bottle_picking/scenario_poses.h>

using edi_bottle_picking::ScenarioPoses;
using edi_bottle_picking::apply_pose_overrides;
using edi_bottle_picking::load_scenario_poses;

namespace
{

rclcpp::Node::SharedPtr make_node(const std::string & name)
{
    static int counter = 0;
    return rclcpp::Node::make_shared(name + "_" + std::to_string(counter++));
}

} // namespace

TEST(ScenarioPoses, DefaultsAreTheRealCellSet)
{
    ScenarioPoses p;
    EXPECT_EQ(p.initial, "near_box");
    EXPECT_EQ(p.above_box, "above_box_2");
    EXPECT_EQ(p.after_pickup, "near_box");
    EXPECT_EQ(p.retreat_fallback, "near_box");
    EXPECT_EQ(p.dp_handoff, "ai_start2");
    EXPECT_EQ(p.dropoff, "inter_floor_4");
}

TEST(ScenarioPoses, YamlOverridesDefaultsAndAbsentKeysFallBack)
{
    YAML::Node cfg;
    cfg["pose_above_box"] = "above_box_1";
    // pose_initial deliberately absent -> keeps the struct default.
    const ScenarioPoses p = load_scenario_poses(cfg);
    EXPECT_EQ(p.above_box, "above_box_1");
    EXPECT_EQ(p.initial, "near_box");
}

TEST(ScenarioPoses, IsaacPresetSelectsTheLegacyEdiIsaacsimBox)
{
    auto node = make_node("isaac_preset");
    node->declare_parameter("pose_set", std::string("isaac"));
    ScenarioPoses p;
    apply_pose_overrides(node, p, "test");
    EXPECT_EQ(p.initial, "wait_slam");
    EXPECT_EQ(p.above_box, "above_box_1");
    EXPECT_EQ(p.after_pickup, "ai_after_pickup");
    EXPECT_EQ(p.retreat_fallback, "wait_slam");
    // ai_start2 is canonical for the DP policy in BOTH cells and must survive the preset.
    EXPECT_EQ(p.dp_handoff, "ai_start2");
}

TEST(ScenarioPoses, EdiPresetRestoresTheRealCellSetOverYaml)
{
    auto node = make_node("edi_preset");
    node->declare_parameter("pose_set", std::string("edi"));
    ScenarioPoses p;
    p.above_box = "above_box_1";  // as if the YAML had pinned the legacy pose
    apply_pose_overrides(node, p, "test");
    EXPECT_EQ(p.above_box, "above_box_2");
}

TEST(ScenarioPoses, EmptyPoseSetLeavesConfiguredPosesAlone)
{
    auto node = make_node("no_preset");
    ScenarioPoses p;
    p.above_box = "og_above_box";
    apply_pose_overrides(node, p, "test");
    EXPECT_EQ(p.above_box, "og_above_box");
}

TEST(ScenarioPoses, UnrecognisedPoseSetIsIgnoredNotFatal)
{
    auto node = make_node("bogus_preset");
    node->declare_parameter("pose_set", std::string("bogus"));
    ScenarioPoses p;
    p.above_box = "og_above_box";
    apply_pose_overrides(node, p, "test");
    EXPECT_EQ(p.above_box, "og_above_box");
}

TEST(ScenarioPoses, PerPoseParamWinsOverThePreset)
{
    auto node = make_node("mixed");
    node->declare_parameter("pose_set", std::string("isaac"));
    node->declare_parameter("pose_above_box", std::string("og_above_box"));
    ScenarioPoses p;
    apply_pose_overrides(node, p, "test");
    EXPECT_EQ(p.above_box, "og_above_box");   // per-pose override applied after the preset
    EXPECT_EQ(p.initial, "wait_slam");        // rest of the preset still in effect
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int rc = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return rc;
}
