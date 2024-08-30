#ifndef HYY_EXTERNAL_CONTROLLER__VISIBILITY_CONTROL_H_
#define HYY_EXTERNAL_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HYY_EXTERNAL_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define HYY_EXTERNAL_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define HYY_EXTERNAL_CONTROLLER_EXPORT __declspec(dllexport)
    #define HYY_EXTERNAL_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef HYY_EXTERNAL_CONTROLLER_BUILDING_LIBRARY
    #define HYY_EXTERNAL_CONTROLLER_PUBLIC HYY_EXTERNAL_CONTROLLER_EXPORT
  #else
    #define HYY_EXTERNAL_CONTROLLER_PUBLIC HYY_EXTERNAL_CONTROLLER_IMPORT
  #endif
  #define HYY_EXTERNAL_CONTROLLER_PUBLIC_TYPE HYY_EXTERNAL_CONTROLLER_PUBLIC
  #define HYY_EXTERNAL_CONTROLLER_LOCAL
#else
  #define HYY_EXTERNAL_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define HYY_EXTERNAL_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define HYY_EXTERNAL_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define HYY_EXTERNAL_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HYY_EXTERNAL_CONTROLLER_PUBLIC
    #define HYY_EXTERNAL_CONTROLLER_LOCAL
  #endif
  #define HYY_EXTERNAL_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // HYY_EXTERNAL_CONTROLLER__VISIBILITY_CONTROL_H_
