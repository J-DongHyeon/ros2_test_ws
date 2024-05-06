#include <pluginlib/class_loader.hpp>
#include "base_class_plugin/base_class_plugin.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;


class CustomPluginTestNode : public rclcpp::Node
{
private:
  std::shared_ptr<base_class_plugin::BaseClassPlugin> base_class_plugin_;
  rclcpp::TimerBase::SharedPtr timer_;


public:
  CustomPluginTestNode(std::shared_ptr<base_class_plugin::BaseClassPlugin> base_class_plugin)
  : Node("plugin_test_node"),
  base_class_plugin_(base_class_plugin)
  {
    RCLCPP_INFO(this->get_logger(), "Plugin Test Node Activated !!!");


    timer_ = this->create_wall_timer(1s, std::bind(&CustomPluginTestNode::run_plugin_action, this));
  }


private:
  void run_plugin_action()
  {
    RCLCPP_INFO(this->get_logger(), "Plugin Name: %s", base_class_plugin_->getName().c_str());


    base_class_plugin_->action();
  }
};


int main(int argc, char* argv[])
{
  pluginlib::ClassLoader<base_class_plugin::BaseClassPlugin> plugin_loader("base_class_plugin",
                                                            "base_class_plugin::BaseClassPlugin");


  std::shared_ptr<base_class_plugin::BaseClassPlugin> plugin =
    plugin_loader.createSharedInstance("custom_plugin::CustomPlugin");


  rclcpp::init(argc, argv);

  auto node = std::make_shared<CustomPluginTestNode>(plugin);


  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
