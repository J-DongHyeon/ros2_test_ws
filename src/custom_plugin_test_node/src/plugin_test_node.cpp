#include <pluginlib/class_loader.hpp>
#include "base_class_test/base_class_test.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PluginTestNode : public rclcpp::Node
{
private:
  std::shared_ptr<base_class_test::BaseClassTest> base_class_test_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  PluginTestNode(std::shared_ptr<base_class_test::BaseClassTest> base_class_test)
  : Node("plugin_test_node"),
  base_class_test_(base_class_test)
  {
    RCLCPP_INFO(this->get_logger(), "Plugin Test Node Activated !!!");

    timer_ = this->create_wall_timer(1s, std::bind(&PluginTestNode::run_plugin_action, this));
  }

private:
  void run_plugin_action()
  {
    RCLCPP_INFO(this->get_logger(), "Plugin Name: %s", base_class_test_->getName().c_str());

    base_class_test_->action();
  }
};

int main(int argc, char* argv[])
{
  pluginlib::ClassLoader<base_class_test::BaseClassTest> plugin_loader("base_class_test",
                                                            "base_class_test::BaseClassTest");

  std::shared_ptr<base_class_test::BaseClassTest> plugin =
    plugin_loader.createSharedInstance("custom_plugin_test::CustomPluginTest");

  rclcpp::init(argc, argv);

  auto node = std::make_shared<PluginTestNode>(plugin);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
