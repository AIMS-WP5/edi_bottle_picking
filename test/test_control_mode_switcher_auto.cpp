// ControlModeSwitcher "auto" position controller against a fake controller_manager (real ROS
// services, in-process): the resolved name must reach the STRICT switch requests, an explicit
// name must skip the listing, and an unresolvable listing must fail without switching.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <edi_bottle_picking/control_mode_switcher.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::SwitchController;

namespace
{

using Listing = std::vector<std::pair<std::string, std::string>>;   // (name, state)

class FakeControllerManager
{
public:
    explicit FakeControllerManager(Listing listing)
    : listing_(std::move(listing))
    {
        node_ = std::make_shared<rclcpp::Node>("fake_controller_manager");
        list_srv_ = node_->create_service<ListControllers>(
            "/controller_manager/list_controllers",
            [this](const std::shared_ptr<ListControllers::Request>,
                   std::shared_ptr<ListControllers::Response> res) {
                ++list_calls;
                for (const auto & [name, state] : listing_) {
                    controller_manager_msgs::msg::ControllerState c;
                    c.name = name;
                    c.state = state;
                    res->controller.push_back(c);
                }
            });
        switch_srv_ = node_->create_service<SwitchController>(
            "/controller_manager/switch_controller",
            [this](const std::shared_ptr<SwitchController::Request> req,
                   std::shared_ptr<SwitchController::Response> res) {
                std::lock_guard<std::mutex> lock(mutex);
                switches.emplace_back(req->activate_controllers, req->deactivate_controllers);
                res->ok = true;
            });
        executor_.add_node(node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    ~FakeControllerManager()
    {
        executor_.cancel();
        spin_thread_.join();
    }

    std::atomic<int> list_calls{0};
    std::mutex mutex;
    std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>> switches;

private:
    Listing listing_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<ListControllers>::SharedPtr list_srv_;
    rclcpp::Service<SwitchController>::SharedPtr switch_srv_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
};

rclcpp::Node::SharedPtr make_client_node()
{
    static int counter = 0;
    return std::make_shared<rclcpp::Node>("switcher_client_" + std::to_string(counter++));
}

}  // namespace

class ControlModeSwitcherAuto : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        // Private domain: the fake serves the real /controller_manager/* names, so it must never
        // share a domain with a live stack (edi_ros_env.sh exports ROS_DOMAIN_ID) or with the
        // other package's copy of this test running in parallel under colcon test.
        rclcpp::InitOptions options;
        options.set_domain_id(TEST_ROS_DOMAIN_ID);
        rclcpp::init(0, nullptr, options);
    }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ControlModeSwitcherAuto, ResolvesScaledAndUsesItInBothSwitches)
{
    FakeControllerManager cm(Listing{{"joint_state_broadcaster", "active"},
                              {"scaled_joint_trajectory_controller", "active"},
                              {"forward_velocity_controller", "inactive"}});
    edi_bottle_picking::ControlModeSwitcher sw(make_client_node(), /*is_isaac=*/false, "auto");

    ASSERT_TRUE(sw.to_velocity_control());
    ASSERT_TRUE(sw.to_position_control());

    EXPECT_EQ(cm.list_calls.load(), 1);   // resolved once, then cached
    std::lock_guard<std::mutex> lock(cm.mutex);
    ASSERT_EQ(cm.switches.size(), 2u);
    EXPECT_EQ(cm.switches[0].first, std::vector<std::string>{"forward_velocity_controller"});
    EXPECT_EQ(cm.switches[0].second,
              std::vector<std::string>{"scaled_joint_trajectory_controller"});
    EXPECT_EQ(cm.switches[1].first,
              std::vector<std::string>{"scaled_joint_trajectory_controller"});
    EXPECT_EQ(cm.switches[1].second, std::vector<std::string>{"forward_velocity_controller"});
}

TEST_F(ControlModeSwitcherAuto, EmptyNameMeansAutoAndResolvesPlain)
{
    FakeControllerManager cm(Listing{{"joint_trajectory_controller", "active"},
                              {"forward_velocity_controller", "inactive"}});
    edi_bottle_picking::ControlModeSwitcher sw(make_client_node(), /*is_isaac=*/false, "");

    ASSERT_TRUE(sw.to_velocity_control());
    std::lock_guard<std::mutex> lock(cm.mutex);
    ASSERT_EQ(cm.switches.size(), 1u);
    EXPECT_EQ(cm.switches[0].second, std::vector<std::string>{"joint_trajectory_controller"});
}

TEST_F(ControlModeSwitcherAuto, ExplicitNameSkipsListing)
{
    FakeControllerManager cm(Listing{{"scaled_joint_trajectory_controller", "active"}});
    edi_bottle_picking::ControlModeSwitcher sw(make_client_node(), /*is_isaac=*/false,
                                               "joint_trajectory_controller");

    ASSERT_TRUE(sw.to_velocity_control());   // the fake accepts any switch
    EXPECT_EQ(cm.list_calls.load(), 0);
    std::lock_guard<std::mutex> lock(cm.mutex);
    ASSERT_EQ(cm.switches.size(), 1u);
    EXPECT_EQ(cm.switches[0].second, std::vector<std::string>{"joint_trajectory_controller"});
}

TEST_F(ControlModeSwitcherAuto, AmbiguousListingFailsWithoutSwitching)
{
    FakeControllerManager cm(Listing{{"scaled_joint_trajectory_controller", "inactive"},
                              {"joint_trajectory_controller", "inactive"}});
    edi_bottle_picking::ControlModeSwitcher sw(make_client_node(), /*is_isaac=*/false, "auto");

    EXPECT_FALSE(sw.to_velocity_control());
    std::lock_guard<std::mutex> lock(cm.mutex);
    EXPECT_TRUE(cm.switches.empty());
}
