#ifndef BASE_CLASS_TEST__BASE_CLASS_TEST_HPP_
#define BASE_CLASS_TEST__BASE_CLASS_TEST_HPP_

#include <string>


namespace base_class_test
{
  class BaseClassTest
  {
    protected:
      std::string name_;

    public:
      BaseClassTest(std::string name = "")
      : name_(name)
      {}

      virtual std::string getName()
      {
        return name_;
      }

      virtual void action() = 0;
  };
}

#endif // !BASE_CLASS_TEST__BASE_CLASS_TEST_HPP_
