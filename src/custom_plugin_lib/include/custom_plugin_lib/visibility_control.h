#ifndef CUSTOM_PLUGIN_TEST__VISIBILITY_CONTROL_H_
#define CUSTOM_PLUGIN_TEST__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CUSTOM_PLUGIN_TEST_EXPORT __attribute__ ((dllexport))
    #define CUSTOM_PLUGIN_TEST_IMPORT __attribute__ ((dllimport))
  #else
    #define CUSTOM_PLUGIN_TEST_EXPORT __declspec(dllexport)
    #define CUSTOM_PLUGIN_TEST_IMPORT __declspec(dllimport)
  #endif
  #ifdef CUSTOM_PLUGIN_TEST_BUILDING_LIBRARY
    #define CUSTOM_PLUGIN_TEST_PUBLIC CUSTOM_PLUGIN_TEST_EXPORT
  #else
    #define CUSTOM_PLUGIN_TEST_PUBLIC CUSTOM_PLUGIN_TEST_IMPORT
  #endif
  #define CUSTOM_PLUGIN_TEST_PUBLIC_TYPE CUSTOM_PLUGIN_TEST_PUBLIC
  #define CUSTOM_PLUGIN_TEST_LOCAL
#else
  #define CUSTOM_PLUGIN_TEST_EXPORT __attribute__ ((visibility("default")))
  #define CUSTOM_PLUGIN_TEST_IMPORT
  #if __GNUC__ >= 4
    #define CUSTOM_PLUGIN_TEST_PUBLIC __attribute__ ((visibility("default")))
    #define CUSTOM_PLUGIN_TEST_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CUSTOM_PLUGIN_TEST_PUBLIC
    #define CUSTOM_PLUGIN_TEST_LOCAL
  #endif
  #define CUSTOM_PLUGIN_TEST_PUBLIC_TYPE
#endif

#endif  // CUSTOM_PLUGIN_TEST__VISIBILITY_CONTROL_H_
