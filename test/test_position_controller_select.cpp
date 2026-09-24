// Pins the "auto" position-controller resolution used by ControlModeSwitcher's DP switch.
// edi_ur loads exactly one of the two trajectory controllers per back end; picking the
// unloaded one makes the STRICT switch fail and aborts the DP segment.

#include <gtest/gtest.h>
#include <edi_bottle_picking/position_controller_select.h>

using edi_bottle_picking::select_position_controller;

namespace
{
const std::string kScaled = "scaled_joint_trajectory_controller";
const std::string kPlain = "joint_trajectory_controller";
}  // namespace

TEST(PositionControllerSelect, OnlyScaledActive)
{
    auto c = select_position_controller({{"joint_state_broadcaster", "active"},
                                         {kScaled, "active"},
                                         {"forward_velocity_controller", "inactive"}});
    EXPECT_EQ(c.name, kScaled);
    EXPECT_EQ(c.state, "active");
    EXPECT_TRUE(c.error.empty());
}

TEST(PositionControllerSelect, OnlyPlainActive)
{
    auto c = select_position_controller({{kPlain, "active"},
                                         {"forward_velocity_controller", "inactive"}});
    EXPECT_EQ(c.name, kPlain);
    EXPECT_TRUE(c.error.empty());
}

TEST(PositionControllerSelect, ScaledInactiveStillChosen)
{
    // controller_stopper keeps the scaled JTC inactive until the robot program plays.
    auto c = select_position_controller({{kScaled, "inactive"},
                                         {"forward_velocity_controller", "inactive"}});
    EXPECT_EQ(c.name, kScaled);
    EXPECT_EQ(c.state, "inactive");
}

TEST(PositionControllerSelect, VelocityActiveIgnored)
{
    // Mid-DP-segment listing: the velocity controller is active, the JTC is not.
    auto c = select_position_controller({{"forward_velocity_controller", "active"},
                                         {kPlain, "inactive"}});
    EXPECT_EQ(c.name, kPlain);
}

TEST(PositionControllerSelect, OneActiveOfBothLoaded)
{
    auto c = select_position_controller({{kScaled, "inactive"}, {kPlain, "active"}});
    EXPECT_EQ(c.name, kPlain);
}

TEST(PositionControllerSelect, BothLoadedInactiveIsError)
{
    auto c = select_position_controller({{kScaled, "inactive"}, {kPlain, "inactive"}});
    EXPECT_TRUE(c.name.empty());
    EXPECT_NE(c.error.find("ambiguous"), std::string::npos);
}

TEST(PositionControllerSelect, BothActiveIsError)
{
    auto c = select_position_controller({{kScaled, "active"}, {kPlain, "active"}});
    EXPECT_TRUE(c.name.empty());
    EXPECT_FALSE(c.error.empty());
}

TEST(PositionControllerSelect, NeitherLoadedIsError)
{
    auto c = select_position_controller({{"joint_state_broadcaster", "active"},
                                         {"forward_velocity_controller", "inactive"}});
    EXPECT_TRUE(c.name.empty());
    EXPECT_NE(c.error.find("neither"), std::string::npos);
}
