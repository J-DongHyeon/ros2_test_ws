#include <iostream>
#include "custom_plugin_test/custom_plugin_test.hpp"

namespace custom_plugin_test
{

CustomPluginTest::CustomPluginTest()
: base_class_test::BaseClassTest("plugin test")
{}

void CustomPluginTest::action()
{
  static int num = 0;

  std::cout << "# " << num++ << " Action Plugin Test !!!" << std::endl;
}

}  // namespace custom_plugin_test

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(custom_plugin_test::CustomPluginTest, base_class_test::BaseClassTest)
