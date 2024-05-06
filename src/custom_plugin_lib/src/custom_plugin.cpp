#include <iostream>
#include "custom_plugin_lib/custom_plugin.hpp"

namespace custom_plugin
{

CustomPlugin::CustomPlugin()
: base_class_plugin::BaseClassPlugin("plugin test")
{}

void CustomPlugin::action()
{
  static int num = 0;

  std::cout << "# " << num++ << " Action Plugin Test !!!" << std::endl;
}

}  // namespace custom_plugin

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(custom_plugin::CustomPlugin, base_class_plugin::BaseClassPlugin)
