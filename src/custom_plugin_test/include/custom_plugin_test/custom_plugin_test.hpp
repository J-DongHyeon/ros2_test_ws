#ifndef CUSTOM_PLUGIN_TEST__CUSTOM_PLUGIN_TEST_HPP_
#define CUSTOM_PLUGIN_TEST__CUSTOM_PLUGIN_TEST_HPP_

#include "custom_plugin_test/visibility_control.h"
#include "base_class_test/base_class_test.hpp"

namespace custom_plugin_test
{

class CustomPluginTest : public base_class_test::BaseClassTest
{
public:
  CustomPluginTest();

  virtual void action() override;
};

}  // namespace custom_plugin_test

#endif  // CUSTOM_PLUGIN_TEST__CUSTOM_PLUGIN_TEST_HPP_
