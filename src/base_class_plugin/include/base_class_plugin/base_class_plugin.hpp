#ifndef BASE_CLASS_PLUGIN__BASE_CLASS_PLUGIN_HPP_
#define BASE_CLASS_PLUGIN__BASE_CLASS_PLUGIN_HPP_

#include <string>


namespace base_class_plugin
{
  class BaseClassPlugin
  {
    protected:
      std::string name_;

    public:
      BaseClassPlugin(std::string name = "")
      : name_(name)
      {}

      virtual std::string getName()
      {
        return name_;
      }

      virtual void action() = 0;
  };
}

#endif // !BASE_CLASS_PLUGIN__BASE_CLASS_PLUGIN_HPP_
