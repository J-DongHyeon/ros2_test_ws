#ifndef CUSTOM_PLUGIN_LIB__CUSTOM_PLUGIN_HPP_
#define CUSTOM_PLUGIN_LIB__CUSTOM_PLUGIN_HPP_

#include "custom_plugin_lib/visibility_control.h"
#include "base_class_plugin/base_class_plugin.hpp"

namespace custom_plugin
{

class CustomPlugin : public base_class_plugin::BaseClassPlugin
{
public:
  CustomPlugin();

  virtual void action() override;
};

}  // namespace custom_plugin_lib

#endif  // !CUSTOM_PLUGIN_LIB__CUSTOM_PLUGIN_HPP_
